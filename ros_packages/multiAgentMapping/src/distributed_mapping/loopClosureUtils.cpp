#include "multiAgentMapping/distributed_mapping/distributedMapping.hpp"

void distributedMapping::loopInfoHandler(
    const multi_agent_mapping::msg::LoopInfo::SharedPtr& msg,
    int id
){
    // Situation 1: need to add pointcloud for loop closure verification
    if((int)msg->noise == 999){
        if(msg->robot0 != robot_id){
            return;
        }

        //copy message
        multi_agent_mapping::msg::LoopInfo loop_msg;
        loop_msg.robot0 = msg->robot0;
        loop_msg.robot1 = msg->robot1;
        loop_msg.index0 = msg->index0;
		loop_msg.index1 = msg->index1;
		loop_msg.init_yaw = msg->init_yaw;
		loop_msg.noise = 888.0; // this loop need verification

        // CHECK_LT(loop_msg.index0,keyposes_cloud_6d->size());
		// CHECK_LT(loop_msg.index0,robots[robot_id].keyframe_cloud_array.size());
        if (loop_msg.index0 >= keyposes_cloud_6d->size()) {
            RCLCPP_ERROR(this->get_logger(), "[LoopClosureUtils : loopInfoHandler] -> Index out of bounds: loop_msg.index0=%d, size=%lu", 
                        loop_msg.index0, keyposes_cloud_6d->size());
            throw std::out_of_range("[LoopClosureUtils : loopInfoHandler] -> loop_msg.index0 is out of bounds of keyposes_cloud_6d.");
        }

        if (loop_msg.index0 >= robots[robot_id].keyframe_cloud_array.size()) {
            RCLCPP_ERROR(this->get_logger(), "[LoopClosureUtils : loopInfoHandler] -> Index out of bounds: loop_msg.index0=%d, keyframe_cloud_array size=%lu", 
                        loop_msg.index0, robots[robot_id].keyframe_cloud_array.size());
            throw std::out_of_range("[LoopClosureUtils : loopInfoHandler] -> loop_msg.index0 is out of bounds of keyframe_cloud_array.");
        }

        // filtered pointcloud
		pcl::PointCloud<PointPose3D>::Ptr cloudTemp(new pcl::PointCloud<PointPose3D>());
		*cloudTemp = robots[robot_id].keyframe_cloud_array[loop_msg.index0];
		downsample_filter_for_inter_loop2.setInputCloud(cloudTemp);
		downsample_filter_for_inter_loop2.filter(*cloudTemp);
		pcl::toROSMsg(*cloudTemp, loop_msg.scan_cloud);
		// relative pose
		loop_msg.pose0 = gtsamPoseToTransform(pclPointTogtsamPose3(keyposes_cloud_6d->points[loop_msg.index0]));

		// publish to others for verification
		robots[robot_id].pub_loop_info->publish(loop_msg);
    }

    // Situation 2: need to verify loop closure in this robot
    else if((int)msg->noise == 888){
		if(msg->robot1 != robot_id){
			return;
		}

		RCLCPP_INFO(this->get_logger(), "[LoopClosureUtils : loopInfoHandler] -> [loopInfoHandler(%d)] check loop %d-%d %d-%d.", 
            id, msg->robot0, msg->index0, msg->robot1, msg->index1);

		loop_closures_candidates.push_back(*msg);
	}

    // Situation 3: add verified loop closure
	else
	{
		RCLCPP_INFO(this->get_logger(), "[LoopClosureUtils : loopInfoHandler] -> [loopInfoHandler(%d)] add loop %d-%d %d-%d.", 
            id, msg->robot0, msg->index0, msg->robot1, msg->index1);

		// extract loop
		Vector Vector6(6);
		Vector6 << msg->noise, msg->noise, msg->noise, msg->noise, msg->noise, msg->noise;
		noiseModel::Diagonal::shared_ptr loop_noise = noiseModel::Diagonal::Variances(Vector6);
		Pose3 pose_between = transformToGtsamPose(msg->pose_between);
		NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(
			Symbol('a'+msg->robot0, msg->index0), Symbol('a'+msg->robot1, msg->index1), pose_between, loop_noise));
		
		// update adjacency matrix
		adjacency_matrix(msg->robot0, msg->robot1) += 1;
		adjacency_matrix(msg->robot1, msg->robot0) += 1;
		if(msg->robot0 == robot_id || msg->robot1 == robot_id)
		{
			// add loop factor
			local_pose_graph->add(factor);
			local_pose_graph_no_filtering->add(factor);
			// enable distributed mapping
			sent_start_optimization_flag = true;

			// update pose estimate (for PCM)
			Key key;
			graph_utils::PoseWithCovariance pose;
			pose.covariance_matrix = loop_noise->covariance();
			pose.pose = transformToGtsamPose(msg->pose1);
			key = Symbol('a'+msg->robot1, msg->index1).key();
			updatePoseEstimateFromNeighbor(msg->robot1, key, pose);
			pose.pose = transformToGtsamPose(msg->pose0);
			key = Symbol('a'+msg->robot0, msg->index0).key();
			updatePoseEstimateFromNeighbor(msg->robot0, key, pose);

            RCLCPP_INFO(this->get_logger(),
                "[LoopClosureUtils] Updating pose estimate from neighbor:\n"
                "  Robot: %d\n"
                "  Key: %lu\n"
                "  Pose: x=%.2f, y=%.2f, z=%.2f",
                msg->robot0,
                key,
                pose.pose.x(),
                pose.pose.y(),
                pose.pose.z());

			// add transform to local map (for PCM)
			auto new_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);

            RCLCPP_INFO(this->get_logger(),
                "[LoopClosureUtils] Added transform to local map:\n"
                "  From robot: %d (key: %lu)\n"
                "  To robot: %d (key: %lu)",
                msg->robot0, new_factor->key1(),
                msg->robot1, new_factor->key2());
			Matrix covariance_matrix = loop_noise->covariance();
			robot_local_map.addTransform(*new_factor, covariance_matrix);

            RCLCPP_INFO(this->get_logger(),
                "[LoopClosureUtils] Transform details after adding to local map:\n"
                "  Total transforms in local map: %zu\n"
                "  Latest transform details:\n"
                "    From: Robot %d (key: %lu)\n"
                "    To: Robot %d (key: %lu)\n"
                "    Transform: x=%.2f, y=%.2f, z=%.2f",
                robot_local_map.getTransforms().transforms.size(),
                msg->robot0, new_factor->key1(),
                msg->robot1, new_factor->key2(),
                new_factor->measured().x(),
                new_factor->measured().y(),
                new_factor->measured().z());
		}
	}
}

void distributedMapping::loopFindGlobalNearKeyframes(
	pcl::PointCloud<PointPose3D>::Ptr& near_keyframes,
	const int& key,
	const int& search_num)
{
	// extract near keyframes
	near_keyframes->clear();
	int pose_num = initial_values->size();
	if (pose_num > robots[robot_id].keyframe_cloud_array.size()) {
        RCLCPP_ERROR(this->get_logger(), 
             "[LoopClosureUtils : loopFindGlobalNearKeyframe] -> Index out of bounds: pose_num=%d, keyframe_cloud_array size=%zu for robot_id=%d", 
             pose_num, robots[robot_id].keyframe_cloud_array.size(), robot_id);
        throw std::out_of_range("[LoopClosureUtils : loopFindGlobalNearKeyframe] -> pose_num is out of bounds of keyframe_cloud_array.");
    }
	int add_num = 0;
	for(int i = -search_num; i <= search_num*2; ++i){
		if(add_num >= search_num*2){
			break;
		}

		int key_near = key + i;
		if(key_near < 0 || key_near >= pose_num){
			continue;
		}
		
		*near_keyframes += *transformPointCloud(robots[robot_id].keyframe_cloud_array[key_near],
			initial_values->at<Pose3>(Symbol('a'+robot_id, key_near)));
		add_num++;
	}

	if(near_keyframes->empty()){
		return;
	}
}

/**
 * @brief Finds and extracts nearby keyframes around a specified keyframe index.
 * 
 * This function searches for nearby keyframes within a specified range of indices 
 * relative to the given keyframe and transforms the associated point clouds to 
 * accumulate them into a single point cloud representing the local neighborhood.
 * 
 * @param near_keyframes A point cloud pointer where the transformed nearby keyframes will be stored.
 * @param key The index of the keyframe around which the search is performed.
 * @param search_num The number of keyframes to search before and after the given keyframe.
 */
void distributedMapping::loopFindNearKeyframes(
    pcl::PointCloud<PointPose3D>::Ptr& near_keyframes,
    const int& key,
    const int& search_num)
{
    // Clear the output point cloud to prepare for new data.
    near_keyframes->clear();

    int pose_num = copy_keyposes_cloud_6d->size();  // Total number of keyframes available.

    if (pose_num > robots[robot_id].keyframe_cloud_array.size()) {
        RCLCPP_ERROR(this->get_logger(), 
            "pose_num (%d) exceeds keyframe_cloud_array size (%zu).", 
            pose_num, robots[robot_id].keyframe_cloud_array.size());
        throw std::out_of_range("pose_num is out of bounds of keyframe_cloud_array.");
    }


    // Search for keyframes within the specified range.
    for(int i = -search_num; i <= search_num; ++i){
        int key_near = key + i;  // Calculate the nearby keyframe index.

        // Skip keyframes that are out of bounds.
        if(key_near < 0 || key_near >= pose_num){
            continue;
        }

        // Transform and accumulate the point cloud of the nearby keyframe.
        *near_keyframes += *transformPointCloud(
            robots[robot_id].keyframe_cloud_array[key_near], 
            &copy_keyposes_cloud_6d->points[key_near]
        );
    }

    // If no nearby keyframes were found, simply return.
    if(near_keyframes->empty()){
        return;
    }
}

/**
 * @brief Updates the pose estimate for a given neighbor robot in the distributed mapping system.
 * 
 * This function updates or inserts a pose estimate received from a neighboring robot, ensuring that
 * the trajectory data is properly managed and that start and end pose IDs are updated accordingly.
 * 
 * @param rid The robot ID of the neighboring robot.
 * @param key The key identifying the specific pose within the trajectory.
 * @param pose The pose estimate with its associated covariance.
 */
void distributedMapping::updatePoseEstimateFromNeighbor(
    const int& rid,
    const Key& key,
    const graph_utils::PoseWithCovariance& pose)
{
    graph_utils::TrajectoryPose trajectory_pose;  // Initialize a new trajectory pose.
    trajectory_pose.id = key;                     // Set the pose ID.
    trajectory_pose.pose = pose;                  // Set the pose with covariance.

    // Check if a trajectory already exists for the given robot ID.
    if(pose_estimates_from_neighbors.find(rid) != pose_estimates_from_neighbors.end()){
        // If the pose already exists, update it.
        if(pose_estimates_from_neighbors.at(rid).trajectory_poses.find(key) != pose_estimates_from_neighbors.at(rid).trajectory_poses.end()){
            pose_estimates_from_neighbors.at(rid).trajectory_poses.at(key) = trajectory_pose;
        }
        // If the pose is new, insert it into the trajectory.
        else {
            pose_estimates_from_neighbors.at(rid).trajectory_poses.insert(make_pair(key, trajectory_pose));
            
            // Update the start and end IDs of the trajectory if necessary.
            if(key < pose_estimates_from_neighbors.at(rid).start_id){
                pose_estimates_from_neighbors.at(rid).start_id = key;
            }
            if(key > pose_estimates_from_neighbors.at(rid).end_id){
                pose_estimates_from_neighbors.at(rid).end_id = key;
            }
        }
    }
    // If no trajectory exists for the robot ID, create a new one.
    else {
        graph_utils::Trajectory new_trajectory;  // Initialize a new trajectory.
        new_trajectory.start_id = key;  // Set the start ID to the current key.
        new_trajectory.end_id = key;    // Set the end ID to the current key.
        new_trajectory.trajectory_poses.insert(make_pair(key, trajectory_pose));  // Insert the pose.
        // Insert the new trajectory into the pose estimates map.
        pose_estimates_from_neighbors.insert(make_pair(rid, new_trajectory));
    
        RCLCPP_INFO(this->get_logger(),
            "[updatePoseEstimateFromNeighbor] Created new trajectory for robot %d:\n"
            "  Key: %lu\n"
            "  Start ID: %lu\n"
            "  End ID: %lu",
            rid, key, new_trajectory.start_id, new_trajectory.end_id);
    }
}


/**
 * @brief Detects a loop closure candidate based on spatial proximity to historical keyframes.
 * 
 * This function checks the surrounding keyframes within a specified search radius to identify 
 * a potential loop closure candidate. It ensures the selected candidate is not within a certain 
 * distance threshold of the current keyframe to avoid false positives from recently visited locations.
 *
 * @param cur_ptr Index of the current keyframe being evaluated for loop closure.
 * 
 * @return The index of the detected loop closure keyframe. Returns -1 if no valid loop closure is detected.
 */
int distributedMapping::detectLoopClosureDistance(
    const int& cur_ptr
){
    int loop_key0 = cur_ptr;  // The current keyframe index being evaluated.
    int loop_key1 = -1;       // Initialize loop closure candidate index as invalid.

    // Find the closest historical keyframes within the search radius.
    vector<int> indices;      // Indices of keyframes found within the search radius.
    vector<float> distances;  // Distances to the keyframes found.
    
    // Set the KD-tree input cloud to search for nearby keyframes.
    kdtree_history_keyposes->setInputCloud(copy_keyposes_cloud_3d);
    
    // Perform radius search around the current keyframe.
    kdtree_history_keyposes->radiusSearch(
        copy_keyposes_cloud_3d->points[cur_ptr], 
        search_radius_, 
        indices, 
        distances, 
        0  // Maximum number of neighbors to return (0 means all within radius).
    );

    // Iterate through the found keyframes to select a valid loop closure candidate.
    for(int i = 0; i < (int)indices.size(); ++i){
        int index = indices[i];  // Get the index of the candidate keyframe.

        // Ensure the candidate is not a recent keyframe to avoid false positives.
        if(loop_key0 > exclude_recent_frame_num_ + index){
            loop_key1 = index;  // Valid loop closure candidate found.
            break;  // Exit the loop once a valid candidate is identified.
        }
    } 

    // If no valid loop closure candidate is found, return -1.
    if(loop_key1 == -1 || loop_key0 == loop_key1){
        return -1;
    }

    // Return the index of the detected loop closure keyframe.
    return loop_key1;
}

void distributedMapping::loopClosureThread(){
    // Terminate the thread if neither intra-robot nor inter-robot loop closures are enabled.
    if(!intra_robot_loop_closure_enable_ && !inter_robot_loop_closure_enable_){
        return;
    }
    RCLCPP_INFO(this->get_logger(), "+++++++++++++++++++++++++++++++++");
	RCLCPP_INFO(this->get_logger(), "Running LOOP_CLOSURE_THREAD");

    // Set the loop rate based on the configured loop closure processing interval.
    rclcpp::Rate rate(1.0 / loop_closure_process_interval_);

    // Main loop for detecting inter-robot loop closures.
    while(rclcpp::ok()){
        rate.sleep();  // Sleep to maintain the desired loop rate.

        // Find intra-loop closures by searching for nearby keyframes within a specified radius.
        performRadiusSearchIntraLoopClosure();

		performDescriptorBasedIntraLoopClosure();

        // Find inter-loop closures 
        performInterLoopClosure();

        // Validate the inter-robot-loop closure
        performExternLoopClosure();
    }
}
