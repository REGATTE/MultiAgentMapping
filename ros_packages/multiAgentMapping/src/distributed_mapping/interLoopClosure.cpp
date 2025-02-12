#include "multiAgentMapping/distributed_mapping/distributedMapping.hpp"

/**
 * @brief Performs inter-loop closure detection between robots in the distributed mapping system.
 * 
 * This function attempts to find loop closure candidates between keyframes of different robots using 
 * global descriptors. If a loop closure is detected, it generates and publishes a LoopInfo message 
 * containing the relevant data to enhance multi-agent mapping accuracy.
 * 
 * Steps:
 * 1. Check if inter-loop closure is enabled and if there are enough keyframes for processing.
 * 2. Find loop closure candidates using place recognition with global descriptors.
 * 3. Validate and log the loop closure if found.
 * 4. Prepare and populate the LoopInfo message, including the scan cloud and pose data when necessary.
 * 5. Publish the LoopInfo message for distributed loop closure processing.
 */
void distributedMapping::performInterLoopClosure(){
    // Check if inter-loop closure is enabled or if we have exceeded the keyframe descriptor size
    if(keyframe_descriptor->getSize() <= inter_robot_loop_ptr || !inter_robot_loop_closure_enable_){
		return;
	}

	RCLCPP_INFO(this->get_logger(), "Performing inter loop closure function initiated!");

    // Step 1: Perform place recognition to find a candidate loop closure
    RCLCPP_INFO(this->get_logger(), "[InterLoopClosure : performInterLoopClosure] -> find loop closure ID")
    auto matching_result = keyframe_descriptor->detectInterLoopClosureID(inter_robot_loop_ptr);
    // Retrieve robot and keyframe indices for the detected loop closure
    int loop_robot0 = keyframe_descriptor->getIndex(inter_robot_loop_ptr).first;
    int loop_robot1 = keyframe_descriptor->getIndex(matching_result.first).first;
    int loop_key0 = keyframe_descriptor->getIndex(inter_robot_loop_ptr).second;
    int loop_key1 = keyframe_descriptor->getIndex(matching_result.first).second;
    float init_yaw = matching_result.second;
    inter_robot_loop_ptr++; // Move to the next keyframe for future inter-loop closure detection

    // Step 2: If no loop closure is found, exit the function
    if(matching_result.first < 0){
        return;
    }

    RCLCPP_INFO(
        this->get_logger(), 
        "[InterLoop<%d>] found between [%d]-[%d][%d] and [%d]-[%d][%d].", 
        robot_id, loop_robot0, loop_key0, inter_robot_loop_ptr - 1, loop_robot1, loop_key1, matching_result.first
    );

    // Step 3: Prepare the LoopInfo message
    multi_agent_mapping::msg::LoopInfo inter_loop_candidate;
    inter_loop_candidate.robot0 = loop_robot0;  // First robot involved in the loop
    inter_loop_candidate.robot1 = loop_robot1;  // Second robot involved in the loop
    inter_loop_candidate.index0 = loop_key0;    // Keyframe index on robot 0
    inter_loop_candidate.index1 = loop_key1;    // Keyframe index on robot 1
    inter_loop_candidate.init_yaw = init_yaw;   // Initial yaw estimate for the loop closure

    // Step 4: Handle noise estimation and scan cloud data preparation
    if(loop_robot0 != robot_id){
        inter_loop_candidate.noise = 999.0; // If the loop is detected with another robot, assign a high noise value
    } else {
        // Check bounds for keyframe and point cloud arrays to avoid access errors
        if (loop_key0 >= keyposes_cloud_6d->size()) {
            RCLCPP_ERROR(this->get_logger(), "loop_key0 out of bounds: %d >= %zu", loop_key0, keyposes_cloud_6d->size());
            return;
        }

        if (loop_key0 >= robots[loop_robot0].keyframe_cloud_array.size()) {
            RCLCPP_ERROR(this->get_logger(), "loop_key0 out of bounds in keyframe cloud array: %d >= %zu", 
                        loop_key0, robots[loop_robot0].keyframe_cloud_array.size());
            return;
        }

		
        // Assign a default noise value (adjust based on confidence in loop closures)
		inter_loop_candidate.noise = 888.0;

        // Create and downsample the keyframe point cloud for efficient processing
		pcl::PointCloud<PointPose3D>::Ptr scan_cloud(new pcl::PointCloud<PointPose3D>());
		pcl::PointCloud<PointPose3D>::Ptr scan_cloud_ds(new pcl::PointCloud<PointPose3D>());
		*scan_cloud = robots[loop_robot0].keyframe_cloud_array[loop_key0];

        // Downsample the keyframe point cloud using a predefined filter
		downsample_filter_for_inter_loop3.setInputCloud(scan_cloud);
		downsample_filter_for_inter_loop3.filter(*scan_cloud_ds);

        // Convert the downsampled point cloud to a ROS message and populate the message field
		pcl::toROSMsg(*scan_cloud_ds, inter_loop_candidate.scan_cloud);

        // Populate the pose information based on the keyframe's 6D pose
		inter_loop_candidate.pose0 = gtsamPoseToTransform(pclPointTogtsamPose3(keyposes_cloud_6d->points[loop_key0]));
    }
    // Step 5: Publish the LoopInfo message for further processing
    robots[robot_id].pub_loop_info->publish(inter_loop_candidate);
}   
/**
 * @brief Perform external loop closure verification and add verified loop closures to the pose graph.
 * 
 * This function handles the verification of loop closures between robots in a distributed mapping system.
 * It uses techniques - ICP (Iterative Closest Point) and RANSAC to validate the correspondences and transform
 * estimation before updating the local pose graph and distributed map.
 * 
 * Main Steps:
 * 1. Extract a loop closure candidate from the queue.
 * 2. Verify that the loop closure has not already been added.
 * 3. Retrieve the initial pose estimates for the loop.
 * 4. Perform cloud extraction and downsampling.
 * 5. Apply ICP and RANSAC to validate the loop closure.
 * 6. Update the pose graph with the loop closure factor.
 * 7. Publish the loop closure and update local maps.
 */
void distributedMapping::performExternLoopClosure(){
    // Check if there are any loop closure candidates and if inter-robot loop closures are enabled
    if(loop_closures_candidates.empty() || !inter_robot_loop_closure_enable_){
		return;
	}
    RCLCPP_INFO(this->get_logger(), "Performing extern inter loop closure function initiated!");

    // Step 1: Extract the loop closure candidate for verification
	multi_agent_mapping::msg::LoopInfo inter_loop = loop_closures_candidates.front();
	loop_closures_candidates.pop_front();

    // Create symbolic representations for the keyframes involved in the loop closure
    auto loop_symbol0 = Symbol('a'+inter_loop.robot0, inter_loop.index0);
	auto loop_symbol1 = Symbol('a'+inter_loop.robot1, inter_loop.index1);

    // Step 2: Check if the loop closure has already been added before
	auto find_key_indexes0 = loop_indexes.find(loop_symbol0);
	auto find_key_indexes1 = loop_indexes.find(loop_symbol1);

    if (find_key_indexes0->second.chr() == loop_symbol1.chr() ||
		find_key_indexes1->second.chr() == loop_symbol0.chr()){
		RCLCPP_ERROR(this->get_logger(), "\033[1;33m[InterLoopClosure : Extern Check] Loop has added. Skip.\033[0m");
		return;
	}

    // Step 3: Fail-safe check - ensure valid keyframe size for loop closure
	if (initial_values->size() < history_keyframe_search_num_*2 || initial_values->size() <= inter_loop.index1){
		loop_closures_candidates.push_back(inter_loop);
		return;
	}

    RCLCPP_INFO(
        this->get_logger(),
        "[InterLoopClosure : performExternLoopClosure<%d>] Loop: %d %d %d %d",
        robot_id, inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1
    );

    // Step 4: Get the initial pose estimate for the keyframe involved in the loop closure
	if (inter_loop.index1 >= initial_values->size()) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "inter_loop.index1 out of bounds: %d >= %zu", 
            inter_loop.index1, initial_values->size()
        );
        return;
    }

	double initial_yaw_;
	if (descriptor_type_num_ == DescriptorType::LidarIris){
		initial_yaw_ = (inter_loop.init_yaw+1)*2*M_PI/60.0;
	}
	else{
		initial_yaw_ = inter_loop.init_yaw*M_PI/180.0;
	}

    // Normalize yaw angle within [-pi, pi]
    if(initial_yaw_ > M_PI){
		initial_yaw_ -= 2*M_PI;
    }
    // Get the initial pose of the loop closure keyframe
    auto initial_loop_pose0 = initial_values->at<Pose3>(loop_symbol1);
    // Calculate the pose of the loop closure considering the initial yaw offset
	auto loop_pose0 = Pose3(
		Rot3::RzRyRx(
			initial_loop_pose0.rotation().roll(),
			initial_loop_pose0.rotation().pitch(),
			initial_loop_pose0.rotation().yaw() + initial_yaw_),
		Point3(
			initial_loop_pose0.translation().x(),
			initial_loop_pose0.translation().y(),
			initial_loop_pose0.translation().z()));

	auto loop_pose1 = initial_values->at<Pose3>(loop_symbol1);

    // Step 5: Extract and transform the scan cloud corresponding to the loop closure
	pcl::PointCloud<PointPose3D>::Ptr scan_cloud_ds(new pcl::PointCloud<PointPose3D>());
	pcl::fromROSMsg(inter_loop.scan_cloud, *scan_cloud_ds);
	*scan_cloud_ds = *transformPointCloud(*scan_cloud_ds, loop_pose0);
	pcl::PointCloud<PointPose3D>::Ptr map_cloud(new pcl::PointCloud<PointPose3D>());
	pcl::PointCloud<PointPose3D>::Ptr map_cloud_ds(new pcl::PointCloud<PointPose3D>());

    // Find nearby keyframes and downsample the point cloud for efficient processing
	loopFindGlobalNearKeyframes(map_cloud, inter_loop.index1, history_keyframe_search_num_);
	downsample_filter_for_inter_loop.setInputCloud(map_cloud); // downsample near keyframes
	downsample_filter_for_inter_loop.filter(*map_cloud_ds);

    // Step 6: Safety checks on the extracted clouds
	if (scan_cloud_ds->size() < 300 || map_cloud_ds->size() < 1000){
		RCLCPP_INFO(this->get_logger(), "[InterLoopClosure : performExternLoopClosure] -> keyFrameCloud too little points 2");
		return;
	}
	if (!scan_cloud_ds->is_dense || !map_cloud_ds->is_dense){
		RCLCPP_INFO(this->get_logger(), "[InterLoopClosure : performExternLoopClosure] -> keyFrameCloud is not dense");
		return;
	}

    // Step 7: Publish the scan and map clouds if subscribers are available
	if(pub_scan_of_scan2map->get_subscription_count() != 0){
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

    /*** Step 8: Calculate transformation using ICP ***/
	static pcl::IterativeClosestPoint<PointPose3D, PointPose3D> icp;
	icp.setMaxCorrespondenceDistance(30);
	icp.setMaximumIterations(100);	
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setRANSACIterations(ransac_maximum_iteration_);
	icp.setRANSACOutlierRejectionThreshold(ransac_outlier_reject_threshold_);

	// Align the clouds using ICP
	icp.setInputSource(scan_cloud_ds);
	icp.setInputTarget(map_cloud_ds);
	pcl::PointCloud<PointPose3D>::Ptr correct_scan_cloud_ds(new pcl::PointCloud<PointPose3D>());
	icp.align(*correct_scan_cloud_ds);
	inter_loop.noise = icp.getFitnessScore();

    /*** Step 9: Verification using RANSAC ***/
    // Estimate correspondences between the aligned clouds
	std::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);
	pcl::registration::CorrespondenceEstimation<PointPose3D, PointPose3D> correspondence_estimation;
	correspondence_estimation.setInputCloud(correct_scan_cloud_ds);
	correspondence_estimation.setInputTarget(map_cloud_ds);
	correspondence_estimation.determineCorrespondences(*correspondences);

	// Apply RANSAC to find inlier correspondences
	pcl::Correspondences new_correspondences;
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointPose3D> correspondence_ransac;
	correspondence_ransac.setInputSource(correct_scan_cloud_ds);
	correspondence_ransac.setInputTarget(map_cloud_ds);
	correspondence_ransac.setMaximumIterations(ransac_maximum_iteration_);
	correspondence_ransac.setInlierThreshold(ransac_outlier_reject_threshold_);
	correspondence_ransac.setInputCorrespondences(correspondences);
	correspondence_ransac.getCorrespondences(new_correspondences);

    // Check if RANSAC passes the outlier threshold
    if (new_correspondences.size() < ransac_threshold_ * correspondences->size()) {
        RCLCPP_DEBUG(
            this->get_logger(), 
            "[InterLoopClosure : performExternLoopClosure] -> \033[1;35m[InterLoop<%d>] [%d][%d]-[%d][%d] RANSAC failed (%.2f < %.2f). Reject.\033[0m",
            robot_id, inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
            new_correspondences.size() * 1.0 / correspondences->size() * 1.0, ransac_threshold_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "[InterLoopClosure : performExternLoopClosure] -> [InterLoop<%d>] RANSAC failed (%.2f < %.2f). Reject.",
            robot_id, new_correspondences.size() * 1.0 / correspondences->size() * 1.0, ransac_threshold_
        );
        return;
    }

    // Check if ICP converges and the noise level is acceptable
    if (icp.hasConverged() == false || inter_loop.noise > fitness_score_threshold_ * 2) {
        RCLCPP_DEBUG(
            this->get_logger(), 
            "[InterLoopClosure : performExternLoopClosure] -> \033[1;35m[InterLoop<%d>] [%d][%d]-[%d][%d] ICP failed (%.2f > %.2f). Reject.\033[0m",
            robot_id, inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
            inter_loop.noise, fitness_score_threshold_ * 2
        );

        RCLCPP_INFO(
            this->get_logger(),
            "[InterLoopClosure : performExternLoopClosure] -> [InterLoop<%d>] ICP failed (%.2f > %.2f). Reject.",
            robot_id, inter_loop.noise, fitness_score_threshold_ * 2
        );
        return;
    }

    RCLCPP_DEBUG(
        this->get_logger(), 
        "[InterLoopClosure : performExternLoopClosure] -> \033[1;35m[InterLoop<%d>] [%d][%d]-[%d][%d] inlier (%.2f < %.2f) fitness (%.2f < %.2f). Add.\033[0m",
        robot_id, inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
        new_correspondences.size() * 1.0 / correspondences->size() * 1.0, ransac_threshold_,
        inter_loop.noise, fitness_score_threshold_ * 2
    );

    RCLCPP_INFO(
        this->get_logger(),
        "[InterLoopClosure : performExternLoopClosure] -> [InterLoop<%d>] inlier (%.2f < %.2f) fitness (%.2f < %.2f). Add.",
        robot_id, 
        new_correspondences.size() * 1.0 / correspondences->size() * 1.0, ransac_threshold_,
        inter_loop.noise, fitness_score_threshold_ * 2
    );

    // Step 10: Calculate pose transformation between keyframes
	auto icp_final_tf = Pose3(icp.getFinalTransformation().cast<double>());
	auto pose_from = icp_final_tf * loop_pose0;
    auto pose_to = loop_pose1;

    inter_loop.pose1 = gtsamPoseToTransform(pclPointTogtsamPose3(keyposes_cloud_6d->points[inter_loop.index1]));
    // Ensure the first robot is always the lower ID to maintain consistency
	if(inter_loop.robot0 > inter_loop.robot1) // the first robot always set to the lower id
	{
		swap(pose_from, pose_to);
		swap(inter_loop.robot0, inter_loop.robot1);
		swap(inter_loop.index0, inter_loop.index1);
		swap(inter_loop.pose0, inter_loop.pose1);
	}
    // Calculate the relative transformation between the two poses
	Pose3 pose_between = pose_from.between(pose_to);
	inter_loop.pose_between = gtsamPoseToTransform(pose_between);

    RCLCPP_INFO(
        this->get_logger(), 
        "[InterLoop<%d>] pose_between: %.6f %.6f %.6f", 
        robot_id, 
        pose_between.translation().x(), 
        pose_between.translation().y(), 
        pose_between.translation().z()
    );

    // Step 11: Create noise model and add factor to the pose graph
	Vector Vector6(6);
	Vector6 << inter_loop.noise, inter_loop.noise, inter_loop.noise, inter_loop.noise, 
		inter_loop.noise, inter_loop.noise;
	noiseModel::Diagonal::shared_ptr loop_noise = noiseModel::Diagonal::Variances(Vector6);
	// add factor
	NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(
		Symbol('a'+inter_loop.robot0, inter_loop.index0),
		Symbol('a'+inter_loop.robot1, inter_loop.index1),
		pose_between, loop_noise));
    
    // Update adjacency matrix and add the factor to the local pose graph
	adjacency_matrix(inter_loop.robot1, inter_loop.robot0) += 1;
	adjacency_matrix(inter_loop.robot0, inter_loop.robot1) += 1;
	local_pose_graph->add(factor);
	local_pose_graph_no_filtering->add(factor);
	sent_start_optimization_flag = true; // enable distributed mapping

    // Step 12: Update pose estimates using the loop closure
	Key key;
	graph_utils::PoseWithCovariance pose;
	pose.covariance_matrix = loop_noise->covariance();
	pose.pose = transformToGtsamPose(inter_loop.pose1);
	key = loop_symbol1.key();
	updatePoseEstimateFromNeighbor(inter_loop.robot1, key, pose);
	pose.pose = transformToGtsamPose(inter_loop.pose0);
	key = loop_symbol0.key();
	updatePoseEstimateFromNeighbor(inter_loop.robot0, key, pose);

    // Step 13: Add the transformation to the local map for further processing
	auto new_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);
	Matrix covariance_matrix = loop_noise->covariance();
	robot_local_map.addTransform(*new_factor, covariance_matrix);

	// Step 14: Publish the verified loop closure
	robots[robot_id].pub_loop_info->publish(inter_loop);
	loop_indexes.emplace(make_pair(loop_symbol0, loop_symbol1));
	loop_indexes.emplace(make_pair(loop_symbol1, loop_symbol0));
}

/**
 * @brief Thread function for handling inter-robot loop closures.
 * 
 * This function runs in a separate thread and continuously checks for intea-loop closures using
 * performInterLoopClosure methods and then validates it using the performExternLoopClosure method, 
 * provided that intra- or inter-loop closure is enabled. It maintains a steady loop rate to balance 
 * processing and resource usage.
 */
void distributedMapping::interLoopClosureThread(){
    // Terminate the thread if neither intra-robot nor inter-robot loop closures are enabled.
    if(!intra_robot_loop_closure_enable_ && !inter_robot_loop_closure_enable_){
        return;
    }
    RCLCPP_INFO(this->get_logger(), "+++++++++++++++++++++++++++++++++");
	RCLCPP_INFO(this->get_logger(), "Running INTER_LOOP_CLOSURE_THREAD");

    // Set the loop rate based on the configured loop closure processing interval.
    rclcpp::Rate rate(1.0 / loop_closure_process_interval_);

    // Main loop for detecting inter-robot loop closures.
    while(rclcpp::ok()){
        rate.sleep();  // Sleep to maintain the desired loop rate.

        // Find inter-loop closures 
        performInterLoopClosure();

        // Validate the inter-robot-loop closure
        performExternLoopClosure();
    }
}
