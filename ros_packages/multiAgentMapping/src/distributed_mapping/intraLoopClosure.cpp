#include "multiAgentMapping/distributed_mapping/distributedMapping.hpp"

/**
 * @brief Performs intra-robot loop closure detection and transformation calculation.
 * 
 * This function attempts to detect loop closures within the keyframe data of a single robot.
 * It performs a radius-based search to find matching keyframes and calculates the necessary 
 * transformation if a valid loop closure is detected.
 */
void distributedMapping::performRadiusSearchIntraLoopClosure(){
    // Check if intra-loop closure processing should continue.
    if(copy_keyposes_cloud_3d->size() <= intra_robot_loop_ptr || intra_robot_loop_closure_enable_){
        return;  // Exit if there are no more keyframes or intra-loop closure is disabled.
    }

    // Find a potential loop closure keyframe using a radius-based search.
    auto matching_result = detectLoopClosureDistance(intra_robot_loop_ptr);
    int loop_key0 = intra_robot_loop_ptr;  // Current keyframe being processed.
    int loop_key1 = matching_result;       // Detected loop closure keyframe.
    intra_robot_loop_ptr++;                // Move to the next keyframe.

    // If no valid loop closure keyframe is found, exit the function.
    if(matching_result < 0) {
        return;
    }

    // Log the detected loop closure between keyframes.
    RCLCPP_INFO(
        this->get_logger(), 
        "[IntraLoopRadiusSearch<%d>] [%d] and [%d].", 
        robot_id, loop_key0, loop_key1
    );

    // Calculate the transformation between the two keyframes to correct the trajectory.
    calculateTransformation(loop_key0, loop_key1);
}

/**
 * @brief Calculates the transformation between two loop closure keyframes using ICP and updates the pose graph.
 * 
 * This function handles loop closure by performing point cloud alignment, verifying the transformation, 
 * and updating the distributed mapping isam2 graph with the calculated transformation, correcting the robots psoe.
 *  It also ensures that  the transformation passes fitness checks before being added to the pose graph.
 * 
 * @param loop_key0 The index of the first keyframe involved in the loop closure.
 * @param loop_key1 The index of the second keyframe involved in the loop closure.
 */
void distributedMapping::calculateTransformation(
    const int& loop_key0,
    const int& loop_key1
){
	// Ensure loop_key0 is within bounds of keyposes cloud.
    if (loop_key0 >= copy_keyposes_cloud_6d->size()) {
        RCLCPP_ERROR(this->get_logger(), "[IntraRobotLoopClosure : calculateTransformation] -> Index out of bounds: loop_key0=%d, copy_keyposes_cloud_6d size=%lu", 
                    loop_key0, copy_keyposes_cloud_6d->size());
        throw std::out_of_range("[IntraRobotLoopClosure : calculateTransformation] -> loop_key0 is out of bounds of copy_keyposes_cloud_6d.");
    }

    // Get the initial poses for the loop closure keyframes.
	Pose3 loop_pose0 = pclPointTogtsamPose3(copy_keyposes_cloud_6d->points[loop_key0]);
	Pose3 loop_pose1 = pclPointTogtsamPose3(copy_keyposes_cloud_6d->points[loop_key1]);

    // Extract and downsample the point clouds for the two keyframes.
	pcl::PointCloud<PointPose3D>::Ptr scan_cloud(new pcl::PointCloud<PointPose3D>());
	pcl::PointCloud<PointPose3D>::Ptr scan_cloud_ds(new pcl::PointCloud<PointPose3D>());
	loopFindNearKeyframes(scan_cloud, loop_key0, 0);
	downsample_filter_for_intra_loop.setInputCloud(scan_cloud);
	downsample_filter_for_intra_loop.filter(*scan_cloud_ds);
	pcl::PointCloud<PointPose3D>::Ptr map_cloud(new pcl::PointCloud<PointPose3D>());
	pcl::PointCloud<PointPose3D>::Ptr map_cloud_ds(new pcl::PointCloud<PointPose3D>());
	loopFindNearKeyframes(map_cloud, loop_key1, history_keyframe_search_num_);
	downsample_filter_for_intra_loop.setInputCloud(map_cloud);
	downsample_filter_for_intra_loop.filter(*map_cloud_ds);

    // Ensure the point clouds have sufficient points.
	if(scan_cloud->size() < 300 || map_cloud->size() < 1000){
		RCLCPP_ERROR(this->get_logger(), "[IntraRobotLoopClosure : calculateTransformation] -> keyFrameCloud too little points 1");
		return;
	}

    // Publish the scan and map point clouds if there are subscribers.
	if(pub_scan_of_scan2map->get_subscription_count() != 0)
	{
		sensor_msgs::msg::PointCloud2 scan_cloud_msg;
		pcl::toROSMsg(*scan_cloud_ds, scan_cloud_msg);
		scan_cloud_msg.header.stamp = this->now();
		scan_cloud_msg.header.frame_id = world_frame_;
		pub_scan_of_scan2map->publish(scan_cloud_msg);
	}
	if(pub_map_of_scan2map->get_subscription_count() != 0)
	{
		sensor_msgs::msg::PointCloud2 map_cloud_msg;
		pcl::toROSMsg(*map_cloud_ds, map_cloud_msg);
		map_cloud_msg.header.stamp = this->now();
		map_cloud_msg.header.frame_id = world_frame_;
		pub_map_of_scan2map->publish(map_cloud_msg);
	}

    // ICP settings for point cloud alignment.
	static pcl::IterativeClosestPoint<PointPose3D, PointPose3D> icp;
	icp.setMaxCorrespondenceDistance(2*search_radius_);
	icp.setMaximumIterations(50);
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setRANSACIterations(0);
	// icp.setRANSACOutlierRejectionThreshold(ransac_outlier_reject_threshold_);

    // Perform point cloud alignment using ICP.
	icp.setInputSource(scan_cloud_ds);
	icp.setInputTarget(map_cloud_ds);
	pcl::PointCloud<PointPose3D>::Ptr unused_result(new pcl::PointCloud<PointPose3D>());
	icp.align(*unused_result);

    // Check if ICP alignment is successful and passes the fitness threshold.
    float fitness_score = icp.getFitnessScore();
    if (icp.hasConverged() == false || fitness_score > fitness_score_threshold_) {
        RCLCPP_DEBUG(this->get_logger(), "[IntraRobotLoopClosure : calculateTransformation] -> \033[1;34m[IntraLoop<%d>] [%d]-[%d] ICP failed (%.2f > %.2f). Reject.\033[0m",
                    robot_id, loop_key0, loop_key1, fitness_score, fitness_score_threshold_);
        RCLCPP_INFO(this->get_logger(), "[IntraRobotLoopClosure : calculateTransformation] -> [IntraLoop<%d>] ICP failed (%.2f > %.2f). Reject.", 
                    robot_id, fitness_score, fitness_score_threshold_);
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "[IntraRobotLoopClosure : calculateTransformation] -> \033[1;34m[IntraLoop<%d>] [%d]-[%d] ICP passed (%.2f < %.2f). Add.\033[0m", 
                robot_id, loop_key0, loop_key1, fitness_score, fitness_score_threshold_);
    RCLCPP_INFO(this->get_logger(), "[IntraRobotLoopClosure : calculateTransformation] -> [IntraLoop<%d>] ICP passed (%.2f < %.2f). Add.", 
                robot_id, fitness_score, fitness_score_threshold_);

    // Get the final transformation pose using ICP results.
	float x, y, z, roll, pitch, yaw;
	Eigen::Affine3f icp_final_tf;
	icp_final_tf = icp.getFinalTransformation();
	pcl::getTranslationAndEulerAngles(icp_final_tf, x, y, z, roll, pitch, yaw);

	// Calculate the corrected pose transformation.
	Eigen::Affine3f origin_tf = gtsamPoseToAffine3f(loop_pose0);
	Eigen::Affine3f correct_tf = icp_final_tf * origin_tf;
	pcl::getTranslationAndEulerAngles(correct_tf, x, y, z, roll, pitch, yaw);

	// Generate the pose difference between the keyframes.
	Pose3 pose_from = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
	Pose3 pose_to = loop_pose1;
    // defined how much the the robots pose at loop_key0 should be adjusted to align with the loop_key1
    // fixes the drift for intra robot loop closure
	Pose3 pose_between = pose_from.between(pose_to);
	RCLCPP_INFO(this->get_logger(), "[IntraRobotLoopClosure : calculateTransformation] -> [IntraLoop<%d>] pose_between: %.2f %.2f %.2f", 
            robot_id, pose_between.translation().x(), pose_between.translation().y(), pose_between.translation().z());

    // Add a loop closure factor to the pose graph.
	Vector vector6(6);
	vector6 << fitness_score, fitness_score, fitness_score, fitness_score, fitness_score, fitness_score;
	noiseModel::Diagonal::shared_ptr loop_noise = noiseModel::Diagonal::Variances(vector6);
	NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(
		Symbol('a'+robot_id, loop_key0), Symbol('a'+robot_id, loop_key1), pose_between, loop_noise));
	isam2_graph.add(factor);
	local_pose_graph->add(factor);
	local_pose_graph_no_filtering->add(factor);
	sent_start_optimization_flag = true; // enable distributed mapping
	intra_robot_loop_close_flag = true; // signals that the intra robot loop closure is detected

    // Save loop factor in the local map for use with PCM.
	auto new_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);
	Matrix covariance = loop_noise->covariance();
	robot_local_map.addTransform(*new_factor, covariance);

	// Track the loop closure in the index map.
	auto it = loop_indexs.find(loop_key0);
	if(it == loop_indexs.end() || (it != loop_indexs.end() && it->second != loop_key1)){
		loop_indexs[loop_key0] = loop_key1;
	}
}

void distributedMapping::performDescriptorBasedIntraLoopClosure()
{
	if(keyframe_descriptor->getSize(robot_id) <= intra_robot_loop_ptr || !intra_robot_loop_closure_enable_){	
		return;
	}

	// find intra loop closure with global descriptor
	auto matching_result = keyframe_descriptor->detectIntraLoopClosureID(intra_robot_loop_ptr);
	int loop_key0 = intra_robot_loop_ptr;
	int loop_key1 = matching_result.first;
	intra_robot_loop_ptr++;

	if(matching_result.first < 0) // no loop found
	{
		return;
	}

	RCLCPP_INFO(this->get_logger(), "[IntraLoop<%d>] [%d] and [%d].", robot_id, loop_key0, loop_key1);

	calculateTransformation(loop_key0, loop_key1);
}

/**
 * @brief Thread function for handling intra-robot loop closures.
 * 
 * This function runs in a separate thread and continuously checks for intra-loop closures using
 * radius-based search methods, provided that intra- or inter-loop closure
 * is enabled. It maintains a steady loop rate to balance processing and resource usage.
 */
void distributedMapping::intraLoopClosureThread(){
    // Terminate the thread if neither intra-robot nor inter-robot loop closures are enabled.
    if(!intra_robot_loop_closure_enable_ && !inter_robot_loop_closure_enable_){
        return;
    }
	RCLCPP_INFO(this->get_logger(), "+++++++++++++++++++++++++++++++++");
	RCLCPP_INFO(this->get_logger(), "Running INTRA_LOOP_CLOSURE_THREAD");

    // Set the loop rate based on the configured loop closure processing interval.
    rclcpp::Rate rate(1.0 / loop_closure_process_interval_);

    // Main loop for detecting intra-robot loop closures.
    while(rclcpp::ok()){
        rate.sleep();  // Sleep to maintain the desired loop rate.

        // Find intra-loop closures by searching for nearby keyframes within a specified radius.
        performRadiusSearchIntraLoopClosure();

		performDescriptorBasedIntraLoopClosure();
    }
}
