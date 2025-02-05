#include "multiAgentMapping/Distributed_Mapping/subGraphsUtils.hpp"
#include "multiAgentMapping/LIO_SAM/utility.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

subGraphMapping::subGraphMapping() : rclcpp::Node("sub_graph_mapping") {
    std::string robot_name = this->get_namespace();
    if(robot_name.length() < 1){
            RCLCPP_ERROR(this->get_logger(), "[subGraphsUtils] -> Invalid robot prefix (should be longer than a letter): %s", robot_name.c_str());
            rclcpp::shutdown();
        }
    robot_namespace = robot_name.substr(1); // Extract the "/"

    // Ensure last character of the name is a digit
    if(!std::isdigit(robot_namespace.back())){
        RCLCPP_ERROR(this->get_logger(), "[subGraphsUtils] -> Invalid namespace format: last character is not a digit!");
        rclcpp::shutdown();
    }
    // extract last char, convert to int and assign as robot_id
    robot_id = robot_namespace.back() - '0';

    // get frames
    this->declare_parameter<std::string>("world_frame", "world");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->get_parameter("world_frame", world_frame_);
    this->get_parameter("odom_frame", odom_frame_);

    if(robot_id != -1){
        singleRobot& robot = robot_info[robot_id];
        robot.robot_namespace = robot_namespace;
        robot.robot_id = robot_id;

        // ros2 time cloud initialization
        robot.point_cloud_input_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
        robot.point_cloud_input_time = 0.0;

        // keyframe cloud initialization
        robot.robot_keyframe_cloud.reset(new pcl::PointCloud<PointPose3D>());
        robot.robot_keyframe_cloud_array.clear();
    }
    //Initializze initial values
    initial_values = std::make_shared<gtsam::Values>();
    #ifdef DEV_MODE
    RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> Robot initialized with namespace: %s, robot ID: %d",
                robot_info[robot_id].robot_namespace.c_str(), robot_info[robot_id].robot_id);
    #endif

    keyposes_cloud_3d.reset(new pcl::PointCloud<PointPose3D>());
    keyposes_cloud_6d.reset(new pcl::PointCloud<PointPose6D>());
    copy_keyposes_cloud_3d.reset(new pcl::PointCloud<PointPose3D>());
    copy_keyposes_cloud_6d.reset(new pcl::PointCloud<PointPose6D>());

    global_path.poses.clear();
	local_path.poses.clear();


    intra_robot_loop_close_flag = false;

    /*** noise model ***/
	odometry_noise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
	prior_noise = noiseModel::Isotropic::Variance(6, 1e-12);

    local_pose_graph = std::make_shared<NonlinearFactorGraph>();
    local_pose_graph_no_filtering = std::make_shared<NonlinearFactorGraph>();  

    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.1;  // Customize as needed
    params.relinearizeSkip = 1;
    isam2 = new gtsam::ISAM2(params);
}

gtsam::Pose3 subGraphMapping::pclPointTogtsamPose3(PointPose6D point){
    return gtsam::Pose3(
        gtsam::Rot3::RzRyRx(
            double(point.roll),
            double(point.pitch),
            double(point.yaw)),
        gtsam::Point3(
            double(point.x),
            double(point.y),
            double(point.z))
        );
    }

/**
 * @brief Processes a keyframe for distributed mapping by adding prior or odometry factors to the pose graph.
 * 
 * This function handles the addition of a keyframe to the distributed mapping system.
 * It either adds a prior factor (if it's the first keyframe) or an odometry factor (if it's not the first keyframe).
 * The keyframe information is stored, and the pose graph is updated accordingly.
 * 
 * @param pose_to The current pose (Pose3) of the robot at the keyframe.
 * @param frame_to The current keyframe's point cloud data.
 * @param timestamp The timestamp associated with the keyframe.
 */
void subGraphMapping::processKeyframeForMapping(
    const Pose3& pose_to,
    const pcl::PointCloud<PointPose3D>::Ptr& frame_to,
    const rclcpp::Time& timestamp) {

        // access the correct robot data using its robot_id
        singleRobot& robot = robot_info[robot_id]; 

        //  === Error checks === 
        // Keyframe pointcloud data is not empty or null
        if (!frame_to || frame_to->empty()) {
            RCLCPP_ERROR(this->get_logger(), "[subGraphsUtils] -> Received an empty or null point cloud for robot id: %d", robot.robot_id);
            return;
        }
        // robot keyframe cloud initialisation check
        if (!robot.robot_keyframe_cloud) {
            RCLCPP_ERROR(this->get_logger(), "[subGraphsUtils] -> Keyframe cloud not properly initialized for robot id: %d", robot.robot_id);
            return;
        }

        // Check if `isam2` was successfully initialized
        if (isam2) {
            RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> ISAM2 successfully initialized.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "[subGraphsUtils] -> Failed to initialize ISAM2.");
        }

        // ===================================================================

        // save keyframe pointcloud into an array
        #ifdef DEV_MODE
        RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> Performing distributed mapping for robot id: %d", robot.robot_id);
        #endif
        pcl::copyPointCloud(*frame_to, *robot.robot_keyframe_cloud);
        robot.robot_keyframe_cloud_array.push_back(*robot.robot_keyframe_cloud);
        #ifdef DEV_MODE
        RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> Keyframe array size for robot id %d: %zu",  robot.robot_id, robot.robot_keyframe_cloud_array.size());
        #endif
        // save timestamp
        robot.point_cloud_input_stamp = timestamp;
        robot.point_cloud_input_time = timestamp.seconds();
        #ifdef DEV_MODE
        RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> Point cloud input stamp: %F", timestamp.seconds());
        #endif

        // Determine current pose symbol
        Pose3 new_pose_to;
        int pose_number = initial_values->size();
        #ifdef DEV_MODE
        RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> Current pose number: %d", pose_number);
        #endif
        Symbol current_symbol = Symbol(robot.robot_id, pose_number);

        if(pose_number == 0) {
            // === First Keyframe: Add Prior Factor ===
            #ifdef DEV_MODE
            RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> Pose number is 0, adding prior factor");
            #endif

            // save prior value for the first keyframe
            robot.prior_odom = pose_to;
            // create and add a prior factor
            auto prior_factor = PriorFactor<Pose3>(current_symbol, pose_to, prior_noise);
            prior_noise = noiseModel::Isotropic::Variance(6, 1e-12);
            #ifdef DEV_MODE
            Eigen::VectorXd sigmas = prior_noise->sigmas();
            RCLCPP_INFO(this->get_logger(), 
                "Prior noise diagonal sigmas: [x: %e, y: %e, z: %e, roll: %e, pitch: %e, yaw: %e]", 
                sigmas(0), sigmas(1), sigmas(2), sigmas(3), sigmas(4), sigmas(5));
            #endif

            local_pose_graph_no_filtering->add(prior_factor);
            isam2_graph.add(prior_factor);
            #ifdef DEV_MODE
            RCLCPP_INFO(this->get_logger(),"[subGraphsUtils] -> Added prior factor to isam2_graph");
            #endif

            // add prior value to the initial values
            initial_values->insert(current_symbol, pose_to);
            isam2_initial_values.insert(current_symbol, pose_to);
            new_pose_to = pose_to;

            // Log data of prior factor
            // The values are aimed at data with minimal to no error 
            RCLCPP_INFO(this->get_logger(), 
                "[subGraphsUtils] -> createPrior: [%d] Translation: [x: %f, y: %f, z: %f] Rotation: [roll: %f, pitch: %f, yaw: %f]",
                robot.robot_id, 
                new_pose_to.translation().x(), 
                new_pose_to.translation().y(), 
                new_pose_to.translation().z(), 
                new_pose_to.rotation().roll(), 
                new_pose_to.rotation().pitch(), 
                new_pose_to.rotation().yaw()
            );
        } else {
            // === Subsequent Keyframes: Add Odometry Factor ===
            #ifdef DEV_MODE
            RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> Pose number is greater that 0, adding odom factor");
            #endif

            // incremental odom in local frame
            // retrieve previous keyframe pose
            auto pose_from = pclPointTogtsamPose3(keyposes_cloud_6d->points[pose_number - 1]);
            // compute pose difference
            auto pose_increment = pose_from.between(pose_to);
            // define symbol for previous pose
            Symbol previous_symbol = Symbol(robot.robot_id, pose_number - 1);
            // retrieve the odometry noise covariance matrix
            Matrix covariance = odometry_noise->covariance();

            // Add odometry factor to graph
            NonlinearFactor::shared_ptr odom_factor(new BetweenFactor<Pose3>(
                previous_symbol, current_symbol, pose_increment, odometry_noise
            ));
            local_pose_graph->add(odom_factor);
            local_pose_graph_no_filtering->add(odom_factor);
            isam2_graph.add(odom_factor);

            // Add odometry value to initial values
            isam2_initial_values.insert(current_symbol, pose_to);
            // Compute the new pose in the global frame
            pose_from = initial_values->at<Pose3>(previous_symbol);
            new_pose_to = pose_from * pose_increment;
            initial_values->insert(current_symbol, new_pose_to);

            // Save factor in local map for PCM
            auto new_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(odom_factor);
            robot_local_map.addTransform(*new_factor, covariance);

            // Log the details of the odometry factor
            RCLCPP_INFO(this->get_logger(), 
            "[subGraphsUtils] -> createOdom:[%d] [%d-%d] -- Previous Pose: [x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f], New Pose: [x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f]",
                robot.robot_id, pose_number - 1, pose_number,
                pose_from.translation().x(), pose_from.translation().y(), pose_from.translation().z(),
                pose_from.rotation().roll(), pose_from.rotation().pitch(), pose_from.rotation().yaw(),
                new_pose_to.translation().x(), new_pose_to.translation().y(), new_pose_to.translation().z(),
                new_pose_to.rotation().roll(), new_pose_to.rotation().pitch(), new_pose_to.rotation().yaw());

        };

        // optimization
        #ifdef DEV_MODE
        isam2_graph.print("[subGraphsUtils] -> GTSAM Graph:\n");
        #endif
        /*
        @brief If the below code fails, the whole of MapOptimization would fail as this will cause a segFault.
        * To fix, make sure isam2 is properly initialised. There is a logger for it above, to check for its initialization status
        */
        isam2->update(isam2_graph, isam2_initial_values);
        isam2_graph.resize(0);
        isam2_initial_values.clear();
        isam2_current_estimate = isam2->calculateEstimate();
        isam2_keypose_estimate = isam2_current_estimate.at<Pose3>(current_symbol);
        #ifdef DEV_MODE
        RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> isam2 updated with isam2_graph and isam2_initial_values");
        #endif

        // save pose in local frame
        static PointPose3D pose_3d;
        pose_3d.x = isam2_keypose_estimate.translation().x();     
        pose_3d.y = isam2_keypose_estimate.translation().y();   
        pose_3d.z = isam2_keypose_estimate.translation().z();   
        pose_3d.intensity = pose_number; // keyframe index
        keyposes_cloud_3d->push_back(pose_3d);
        #ifdef DEV_MODE
        RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> keyposes_cloud_3d updated");
        #endif

        static PointPose6D pose_6d;
        pose_6d.x = pose_3d.x;
        pose_6d.y = pose_3d.y;
        pose_6d.z = pose_3d.z;
        pose_6d.intensity = pose_3d.intensity;
        pose_6d.roll = isam2_keypose_estimate.rotation().roll();
        pose_6d.pitch = isam2_keypose_estimate.rotation().pitch();
        pose_6d.yaw = isam2_keypose_estimate.rotation().yaw();
        pose_6d.time = robot_info[robot_id].point_cloud_input_time; // keyframe timestamp
        keyposes_cloud_6d->push_back(pose_6d);
        #ifdef DEV_MODE
        RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> keyposes_cloud_6d updated");
        #endif

        // save path for visualization
        updateLocalPath(pose_6d);
        #ifdef DEV_MODE
        RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> saved poses in local frame for visualization");
        #endif

        updateGlobalPath(new_pose_to);
        #ifdef DEV_MODE
        RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> saved poses in global frame for visualization");
        #endif
    }

void subGraphMapping::updateLocalPath(const PointPose6D& pose){
    // Create a PoseStamped message
    static geometry_msgs::msg::PoseStamped pose_stamped;

    pose_stamped.header.stamp = rclcpp::Clock().now();
    pose_stamped.header.frame_id = world_frame_;
    pose_stamped.pose.position.x = pose.x;
    pose_stamped.pose.position.y = pose.y;
    pose_stamped.pose.position.z = pose.z;

    tf2::Quaternion q;
    q.setRPY(pose.roll, pose.pitch, pose.yaw);
    pose_stamped.pose.orientation = tf2::toMsg(q);

    // Add the pose to thje local path
    local_path.poses.push_back(pose_stamped);
} 

void subGraphMapping::updateGlobalPath(const gtsam::Pose3& pose) {
    static geometry_msgs::msg::PoseStamped pose_stamped;
    
    pose_stamped.header.stamp = rclcpp::Clock().now();
    pose_stamped.header.frame_id = world_frame_;
    
    // Corrected calls using x(), y(), z() methods explicitly
    pose_stamped.pose.position.x = static_cast<double>(pose.translation().x());
    pose_stamped.pose.position.y = static_cast<double>(pose.translation().y());
    pose_stamped.pose.position.z = static_cast<double>(pose.translation().z());

    pose_stamped.pose.orientation.x = pose.rotation().toQuaternion().x();
    pose_stamped.pose.orientation.y = pose.rotation().toQuaternion().y();
    pose_stamped.pose.orientation.z = pose.rotation().toQuaternion().z();
    pose_stamped.pose.orientation.w = pose.rotation().toQuaternion().w();

    global_path.poses.push_back(pose_stamped);
}

/**
 * @brief Updates the key poses and local path based on current pose estimates.
 *
 * This function updates the robot's key pose point clouds (3D and 6D), local path,
 * and key pose estimates using the latest ISAM2 estimates. If an intra-robot loop
 * closure is detected, it clears the local path and processes the new key poses.
 *
 * @return true if updates were made, false otherwise.
 */
bool subGraphMapping::updatePoses(){
    // access the correct robot data using its robot_id
    singleRobot& robot = robot_info[robot_id]; 
    // Return value indicating if updates were made
    bool return_value = false;

    #ifdef DEV_MODE
    RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> checking if intra robot loop closure has occured for [%d]", robot.robot_id);
    #endif

    // Check if keyposes_cloud_3d is empty
    if(keyposes_cloud_3d->empty()){
        return return_value; // No updates if the 3D key pose cloud is empty
    }
    // If an intra-robot loop closure has been detected, process the updates
    if(intra_robot_loop_close_flag){
        // Clear the local path to prepare for updated key poses
        local_path.poses.clear();

        // Iterate through the current ISAM2 estimates and update key poses
        for (const Values::ConstKeyValuePair &key_value: isam2_current_estimate){
            // Retrieve the key and its corresponding index in the point cloud
            Symbol key = key_value.key;
            int index = key.index();

            // Retrieve the pose corresponding to this key
            Pose3 pose = isam2_current_estimate.at<Pose3>(key);

            // Update the 3D key pose cloud with the new translation
            keyposes_cloud_3d->points[index].x = pose.translation().x();
            keyposes_cloud_3d->points[index].y = pose.translation().y();
            keyposes_cloud_3d->points[index].z = pose.translation().z();

            // Update the 6D key pose cloud with both position and orientation
            keyposes_cloud_6d->points[index].x = keyposes_cloud_3d->points[index].x;
            keyposes_cloud_6d->points[index].y = keyposes_cloud_3d->points[index].y;
            keyposes_cloud_6d->points[index].z = keyposes_cloud_3d->points[index].z;
            keyposes_cloud_6d->points[index].roll = pose.rotation().roll();
            keyposes_cloud_6d->points[index].pitch = pose.rotation().pitch();
            keyposes_cloud_6d->points[index].yaw = pose.rotation().yaw();

            // Add the updated pose to the local path
            updateLocalPath(keyposes_cloud_6d->points[index]);
        }

        // Reset the intra-robot loop closre flag and set return value to true
        intra_robot_loop_close_flag = false;
        return_value = true;
        #ifdef DEV_MODE
        RCLCPP_INFO(this->get_logger(), "[subGraphsUtils] -> updated poses");
        #endif
    }

    // copy the updated keyposes
    *copy_keyposes_cloud_3d = *keyposes_cloud_3d;
    *copy_keyposes_cloud_6d = *keyposes_cloud_6d;

    // update the most recent keypose, representing robots current position
    isam2_keypose_estimate = pclPointTogtsamPose3(keyposes_cloud_6d->back());
    return return_value;
}

/** 
* @brief The poses are updated after the intra-robot loop closure
* Those new poses are saved under isam2_keypose_estimate, which is 
* returned by this function.
* 
* If the poses arent updated by the above function in isam2_keypose_estimate,
* then it returns the current poses saved earlier.
*/
Pose3 subGraphMapping::getNewEstimate(){
    return isam2_keypose_estimate;
}

void subGraphMapping::makeLiDARIrisDescriptors(){

}