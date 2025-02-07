#include "multiAgentMapping/distributed_mapping/paramsServer.hpp"

paramsServer::paramsServer() : Node("params_server_node"){
    // robot info
    std::string robot_namespace = this->get_namespace();
    if(robot_namespace.length() < 1){
        RCLCPP_ERROR(this->get_logger(), "[paramsServer] -> Invalid robot namespace");
        rclcpp::shutdown();
    }
    robot_name = robot_namespace.substr(1); // remove the "/"
    // Ensure last character of the name is a digit
    if(!std::isdigit(robot_namespace.back())){
        RCLCPP_ERROR(this->get_logger(), "[subGraphsUtils] -> Invalid namespace format: last character is not a digit!");
        rclcpp::shutdown();
    }
    // extract last char, convert to int and assign as robot_id
    robot_id = robot_namespace.back() - '0';

    this->declare_parameter<int>("number_of_robots", 1); // declaring the param with default of 1
    this->get_parameter("number_of_robots", number_of_robots_); // Retrieve the parameter value

    if (number_of_robots_ < 1){
        RCLCPP_ERROR(this->get_logger(), "[paramsServer] -> invalid number of robots");
        rclcpp::shutdown();
    }

    robot_namespace += "/distributed_mapping";

    // get frame names
    // Declare and retrieve the world_frame parameter
    this->declare_parameter<std::string>("world_frame", "world"); // default -> world
    this->get_parameter("world_frame", world_frame_);

    // Declare and retrieve the odom_frame parameter
    this->declare_parameter<std::string>("odom_frame", "odom"); // default -> odom
    this->get_parameter("odom_frame", odom_frame_);

    // lidar config
    sensor_ = LiDARType::VELODYNE; // only using velodyne
    this->declare_parameter<int>(robot_namespace + "/n_scan", 128); // default -> 128 [VLS 128 used]
    this->get_parameter(robot_namespace + "/n_scan", n_scan);

    // cpu params
    // Declare parameters with default values
    this->declare_parameter<int>(robot_namespace + "/onboard_cpu_cores_num", 4);  // Default: 4 cores
    this->declare_parameter<float>(robot_namespace + "/loop_closure_process_interval", 0.02);  // Default: 0.02 second
    this->declare_parameter<float>(robot_namespace + "/map_publish_interval", 10.0);  // Default: 10.0 seconds
    this->declare_parameter<float>(robot_namespace + "/mapping_process_interval", 0.1);  // Default: 0.1 seconds

    // Get parameter values
    this->get_parameter(robot_namespace + "/onboard_cpu_cores_num", onboard_cpu_cores_num_);
    this->get_parameter(robot_namespace + "/loop_closure_process_interval", loop_closure_process_interval_);
    this->get_parameter(robot_namespace + "/map_publish_interval", map_publish_interval_);
    this->get_parameter(robot_namespace + "/mapping_process_interval", mapping_process_interval_);

    this->declare_parameter<bool>(robot_namespace + "/global_optimization_enable", false);
    this->get_parameter(robot_namespace + "/global_optimization_enable", global_optimization_enable_);

    this->declare_parameter<bool>(robot_namespace + "/use_pcm", false);
    this->get_parameter(robot_namespace + "/use_pcm", use_pcm_);

    this->declare_parameter<float>(robot_namespace + "/pcm_threshold", 0.75);
    this->get_parameter(robot_namespace + "/pcm_threshold", pcm_threshold_);

    this->declare_parameter<bool>(robot_namespace + "/use_between_noise", false);
    this->get_parameter(robot_namespace + "/use_between_noise", use_between_noise_);

    this->declare_parameter<int>(robot_namespace + "/optmization_maximum_iteration", 100);
    this->get_parameter(robot_namespace + "/optmization_maximum_iteration", optmization_maximum_iteration_);

    this->declare_parameter<float>(robot_namespace + "/failsafe_wait_time", 1.0);
    this->get_parameter(robot_namespace + "/failsafe_wait_time", fail_safe_wait_time_);
    fail_safe_steps_ = fail_safe_wait_time_ / mapping_process_interval_;

    this->declare_parameter<float>(robot_namespace + "/rotation_estimate_change_threshold", 0.1);
    this->get_parameter(robot_namespace + "/rotation_estimate_change_threshold", rotation_estimate_change_threshold_);

    this->declare_parameter<float>(robot_namespace + "/pose_estimate_change_threshold", 0.1);
    this->get_parameter(robot_namespace + "/pose_estimate_change_threshold", pose_estimate_change_threshold_);

    this->declare_parameter<float>(robot_namespace + "/gamma", 1.0);
    this->get_parameter(robot_namespace + "/gamma", gamma_);

    this->declare_parameter<bool>(robot_namespace + "/use_flagged_init", true);
    this->get_parameter(robot_namespace + "/use_flagged_init", use_flagged_init_);

    this->declare_parameter<bool>(robot_namespace + "/use_landmarks", false);
    this->get_parameter(robot_namespace + "/use_landmarks", use_landmarks_);

    this->declare_parameter<bool>(robot_namespace + "/use_heuristics", true);
    this->get_parameter(robot_namespace + "/use_heuristics", use_heuristics_);

    this->declare_parameter<float>(robot_namespace + "/map_leaf_size", 0.4);
    this->get_parameter(robot_namespace + "/map_leaf_size", map_leaf_size_);

    this->declare_parameter<float>(robot_namespace + "/descript_leaf_size", 0.1);
    this->get_parameter(robot_namespace + "/descript_leaf_size", descript_leaf_size_);

    this->declare_parameter<float>(robot_namespace + "/intra_robot_loop_closure_enable", true);
    this->get_parameter(robot_namespace + "/intra_robot_loop_closure_enable", intra_robot_loop_closure_enable_);

    this->declare_parameter<float>(robot_namespace + "/inter_robot_loop_closure_enable", true);
    this->get_parameter(robot_namespace + "/inter_robot_loop_closure_enable", inter_robot_loop_closure_enable_);

    descriptor_type_num_ = DescriptorType::LidarIris;
    RCLCPP_INFO(this->get_logger(), "Descriptor type set to LidarIris.");

    this->declare_parameter<int>(robot_namespace + "/knn_candidates", 10);
    this->declare_parameter<int>(robot_namespace + "/exclude_recent_frame_num", 30);
    this->declare_parameter<float>(robot_namespace + "/search_radius", 15.0);
    // match mode 2 -> iris feature matching
    this->declare_parameter<int>(robot_namespace + "/match_mode", 2);
    this->declare_parameter<int>(robot_namespace + "/iris_row", 80);
    this->declare_parameter<int>(robot_namespace + "/iris_column", 360);
    this->declare_parameter<float>(robot_namespace + "/descriptor_distance_threshold", 0.4);
    this->declare_parameter<int>(robot_namespace + "/history_keyframe_search_num", 16);
    this->declare_parameter<float>(robot_namespace + "/fitness_score_threshold", 0.2);
    this->declare_parameter<int>(robot_namespace + "/ransac_maximum_iteration", 1000);
    this->declare_parameter<float>(robot_namespace + "/ransac_threshold", 0.5);
    this->declare_parameter<float>(robot_namespace + "/ransac_outlier_reject_threshold", 0.05);
    this->get_parameter(robot_namespace + "/knn_candidates", knn_candidates_);
    this->get_parameter(robot_namespace + "/exclude_recent_frame_num", exclude_recent_frame_num_);
    this->get_parameter(robot_namespace + "/search_radius", search_radius_);
    this->get_parameter(robot_namespace + "/match_mode", match_mode_);
    this->get_parameter(robot_namespace + "/iris_row", iris_row_);
    this->get_parameter(robot_namespace + "/iris_column", iris_column_);
    this->get_parameter(robot_namespace + "/descriptor_distance_threshold", descriptor_distance_threshold_);
    this->get_parameter(robot_namespace + "/history_keyframe_search_num", history_keyframe_search_num_);
    this->get_parameter(robot_namespace + "/fitness_score_threshold", fitness_score_threshold_);
    this->get_parameter(robot_namespace + "/ransac_maximum_iteration", ransac_maximum_iteration_);
    this->get_parameter(robot_namespace + "/ransac_threshold", ransac_threshold_);
    this->get_parameter(robot_namespace + "/ransac_outlier_reject_threshold", ransac_outlier_reject_threshold_);

    this->declare_parameter<float>(robot_namespace + "/global_map_visualization_radius", 60.0);
    this->get_parameter(robot_namespace + "/global_map_visualization_radius", global_map_visualization_radius_);

    this->declare_parameter<std::string>(robot_namespace + "/save_directory", "/distributed_mapping_output");
    this->get_parameter(robot_namespace + "/save_directory", save_directory_);
}

// convert gtsam pose3 to eigen affine3f
Eigen::Affine3f paramsServer::gtsamPoseToAffine3f(gtsam::Pose3 pose){
    return pcl::getTransformation(
        pose.translation().x(),
        pose.translation().y(),
        pose.translation().z(),
        pose.rotation().roll(),
        pose.rotation().pitch(),
        pose.rotation().yaw()
    );
}

// convert gtsam pose3 to ros2 geometry msgs transform message
geometry_msgs::msg::Transform paramsServer::gtsamPoseToTransform(gtsam::Pose3 pose){
    geometry_msgs::msg::Transform transform_msg;
    transform_msg.translation.x = pose.translation().x();
    transform_msg.translation.y = pose.translation().y();
    transform_msg.translation.z = pose.translation().z();
    transform_msg.rotation.w = pose.rotation().toQuaternion().w();
    transform_msg.rotation.x = pose.rotation().toQuaternion().x();
    transform_msg.rotation.y = pose.rotation().toQuaternion().y();
    transform_msg.rotation.z = pose.rotation().toQuaternion().z();

    return transform_msg;
}

// Convert ROS 2 geometry_msgs Transform to GTSAM Pose3
gtsam::Pose3 paramsServer::transformToGtsamPose(const geometry_msgs::msg::Transform& pose){
    return gtsam::Pose3(
        gtsam::Rot3::Quaternion(
            pose.rotation.w,
            pose.rotation.x,
            pose.rotation.y,
            pose.rotation.z
        ),
        gtsam::Point3(
            pose.translation.x,
            pose.translation.y,
            pose.translation.z
        )
    );
}

// Convert a custom PointPose6D (position + orientation) to GTSAM Pose3
gtsam::Pose3 paramsServer::pclPointTogtsamPose3(PointPose6D point)
{
	return gtsam::Pose3(
        gtsam::Rot3::RzRyRx(
            double(point.roll), 
            double(point.pitch), 
            double(point.yaw)),
		gtsam::Point3(
            double(point.x), 
            double(point.y), 
            double(point.z)
        )
    );
}

// Transform a point cloud using a given transformation (PointPose6D) and return the transformed cloud
pcl::PointCloud<PointPose3D>::Ptr paramsServer::transformPointCloud(pcl::PointCloud<PointPose3D> cloud_input, PointPose6D* pose)
{
    // Create an output point cloud with the same size as the input cloud
	pcl::PointCloud<PointPose3D>::Ptr cloud_output(new pcl::PointCloud<PointPose3D>());

	int cloud_size = cloud_input.size();
	cloud_output->resize(cloud_size);

    // Compute the transformation matrix using the given pose (position + orientation)
	Eigen::Affine3f trans_cur = pcl::getTransformation(pose->x, pose->y, pose->z, pose->roll, pose->pitch, pose->yaw);
	
    // Parallel processing using OpenMP to transform all points
	#pragma omp parallel for num_threads(onboard_cpu_cores_num_)
	for(int i = 0; i < cloud_size; ++i)
	{
		const auto &p_from = cloud_input.points[i];
		cloud_output->points[i].x = trans_cur(0,0)*p_from.x + trans_cur(0,1)*p_from.y + trans_cur(0,2)*p_from.z + trans_cur(0,3);
		cloud_output->points[i].y = trans_cur(1,0)*p_from.x + trans_cur(1,1)*p_from.y + trans_cur(1,2)*p_from.z + trans_cur(1,3);
		cloud_output->points[i].z = trans_cur(2,0)*p_from.x + trans_cur(2,1)*p_from.y + trans_cur(2,2)*p_from.z + trans_cur(2,3);
		cloud_output->points[i].intensity = p_from.intensity;
	}
	return cloud_output;
}

// Transform a point cloud using a GTSAM Pose3 transformation
pcl::PointCloud<PointPose3D>::Ptr paramsServer::transformPointCloud(pcl::PointCloud<PointPose3D> cloud_input, gtsam::Pose3 pose)
{
    // Create an output point cloud with the same size as the input cloud
	pcl::PointCloud<PointPose3D>::Ptr cloud_output(new pcl::PointCloud<PointPose3D>());

	int cloud_size = cloud_input.size();
	cloud_output->resize(cloud_size);

    // Compute the transformation matrix from the GTSAM Pose3
	Eigen::Affine3f trans_cur = pcl::getTransformation(pose.translation().x(), pose.translation().y(), pose.translation().z(),
		pose.rotation().roll(), pose.rotation().pitch(), pose.rotation().yaw());
	
    // Parallel processing using OpenMP to transform all points
	#pragma omp parallel for num_threads(onboard_cpu_cores_num_)
	for(int i = 0; i < cloud_size; ++i)
	{
		const auto &p_from = cloud_input.points[i];
		cloud_output->points[i].x = trans_cur(0,0)*p_from.x + trans_cur(0,1)*p_from.y + trans_cur(0,2)*p_from.z + trans_cur(0,3);
		cloud_output->points[i].y = trans_cur(1,0)*p_from.x + trans_cur(1,1)*p_from.y + trans_cur(1,2)*p_from.z + trans_cur(1,3);
		cloud_output->points[i].z = trans_cur(2,0)*p_from.x + trans_cur(2,1)*p_from.y + trans_cur(2,2)*p_from.z + trans_cur(2,3);
		cloud_output->points[i].intensity = p_from.intensity;
	}
	return cloud_output;
}