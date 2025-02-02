#include "multiAgentMapping/Distributed_Mapping/subGraphsUtils.hpp"
#include "multiAgentMapping/LIO_SAM/utility.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>

subGraphMapping::subGraphMapping() : rclcpp::Node("sub_graph_mapping") {
    std::string robot_name = this->get_namespace();
    if(robot_name.length() < 1){
            RCLCPP_ERROR(this->get_logger(), "Invalid robot prefix (should be longer than a letter): %s", robot_name.c_str());
            rclcpp::shutdown();
        }
    robot_namespace = robot_name.substr(1); // Extract the "/"

    // Ensure last character of the name is a digit
    if(!std::isdigit(robot_namespace.back())){
        RCLCPP_ERROR(this->get_logger(), "Invalid namespace format: last character is not a digit!");
        rclcpp::shutdown();
    }
    // extract last char, convert to int and assign as robot_id
    robot_id = robot_namespace.back() - '0';

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
    RCLCPP_INFO(this->get_logger(), "Robot initialized with namespace: %s, robot ID: %d",
                robot_info[robot_id].robot_namespace.c_str(), robot_info[robot_id].robot_id);
    #endif
}

void subGraphMapping::performDistributedMapping(
    const Pose3& pose_to,
    const pcl::PointCloud<PointPose3D>::Ptr& frame_to,
    const rclcpp::Time& timestamp) {

        // access the correct robot
        singleRobot& robot = robot_info[robot_id]; 

        // Error checks
        // Keyframe pointcloud data
        if (!frame_to || frame_to->empty()) {
            RCLCPP_ERROR(this->get_logger(), "Received an empty or null point cloud for robot id: %d", robot.robot_id);
            return;
        }
        // robot keyframe cloud initialisation check
        if (!robot.robot_keyframe_cloud) {
            RCLCPP_ERROR(this->get_logger(), "Keyframe cloud not properly initialized for robot id: %d", robot.robot_id);
            return;
        }

        // ===================================================================

        // save keyframe pointcloud into an array
        #ifdef DEV_MODE
        RCLCPP_INFO(this->get_logger(), "Performing distributed mapping for robot id: %d", robot.robot_id);
        #endif
        pcl::copyPointCloud(*frame_to, *robot.robot_keyframe_cloud);
        robot.robot_keyframe_cloud_array.push_back(*robot.robot_keyframe_cloud);
        #ifdef DEV_MODE
        RCLCPP_INFO(this->get_logger(), "Keyframe array size for robot id %d: %zu",  robot.robot_id, robot.robot_keyframe_cloud_array.size());
        #endif
        // save timestamp
        robot.point_cloud_input_stamp = timestamp;
        robot.point_cloud_input_time = timestamp.seconds();
        #ifdef DEV_MODE
        RCLCPP_INFO(this->get_logger(), "Point cloud input stamp: %F", timestamp.seconds());
        #endif

        // Add prior factor
        Pose3 new_pose_to;
        int pose_number = initial_values->size();
        #ifdef DEV_MODE
        RCLCPP_INFO(this->get_logger(), "Current pose number: %d", pose_number);
        #endif
        Symbol current_symbol = Symbol(robot.robot_id, pose_number);
        if(pose_number == 0) {
            #ifdef DEV_MODE
            RCLCPP_INFO(this->get_logger(), "Pose number is 0, adding prior factor");
            #endif

            // save prior value
            robot.prior_odom = pose_to;


        } else {
            #ifdef DEV_MODE
            RCLCPP_INFO(this->get_logger(), "Pose number is greater that 0, adding odom factor");
            #endif
        }

    }