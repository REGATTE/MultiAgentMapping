#include "multiAgentMapping/distributed_mapping/distributedMapping.hpp"

void distributedMapping::globalDescriptorHandler(
	const multi_agent_mapping::msg::GlobalDescriptor::SharedPtr& msg,
	int id){
	// save timestamp
    robots[id].time_cloud_input_stamp = msg->header.stamp;
    robots[id].time_cloud_input = rclcpp::Time(robots[id].time_cloud_input_stamp).seconds();
	// save descriptors
	RCLCPP_INFO(
        rclcpp::get_logger("distributedMapping"), 
        "[globalDescriptorHandler(%d)] saveDescriptorAndKey: %d.", 
        id, msg->index
    );
	// keyframe_descriptor->saveDescriptorAndKey(msg->values.data(), id, msg->index);
	store_descriptors.emplace_back(make_pair(id,*msg));
}

void distributedMapping::optStateHandler(
	const std_msgs::msg::Int8::SharedPtr& msg,
	int id)
{
	neighbors_started_optimization[id] = (OptimizerState)msg->data <= OptimizerState::Start;
	neighbor_state[id] = (OptimizerState)msg->data;
	neighbors_lowest_id_included[id] = lowest_id_included;
	if(neighbors_within_communication_range.find(id) == neighbors_within_communication_range.end())
	{
		neighbors_within_communication_range.insert(id);
	}
}

void distributedMapping::rotationStateHandler(
	const std_msgs::msg::Int8::SharedPtr& msg,
	int id)
{
	neighbors_rotation_estimate_finished[id] = msg->data;
}

void distributedMapping::poseStateHandler(
	const std_msgs::msg::Int8::SharedPtr& msg,
	int id)
{
	neighbors_pose_estimate_finished[id] = msg->data;
	neighbors_estimation_done[id] = msg->data;
}

/**
 * @brief Handles incoming rotation estimates from neighboring robots and updates the local rotation estimation.
 *
 * This function is part of the distributed multi-robot mapping system and is called whenever a neighboring robot
 * sends a rotation estimate message. The function performs the following key tasks:
 *
 * - Validates the incoming message to ensure it is relevant for the current robot and phase of optimization.
 * - Updates the rotation estimates using information from neighboring robots.
 * - Checks whether the optimization has converged or reached the maximum number of iterations.
 * - Sends the updated rotation estimates to the next robot in the optimization order.
 * - Publishes the current state of the rotation estimation to neighboring robots.
 *
 * @param msg The incoming message containing rotation estimates from a neighboring robot.
 * @param id The ID of the neighboring robot that sent the message.
 * @param robot_id The ID of the current robot
 */
void distributedMapping::neighborRotationHandler(
	const multi_agent_mapping::msg::NeighborEstimate::SharedPtr& msg,
	int id)
{
	// Ignore the message if it is not intended for this robot or 
	// if the robot is not in the rotation estimation phase
	if(msg->receiver_id != robot_id || optimizer_state != OptimizerState::RotationEstimation){
		return;
	}

	// update neighbor rotation estimates
	// Iterate through all the pose IDs provided in the incoming message
	for(int i =0; i<msg->pose_id.size(); i++){
		Symbol symbol((id + 'a'), msg->pose_id[i]); // Create a symbol representing the pose of the neighboring robot
		// Check if pose graph optimization (PCM) is disabled or if the pose is part of the current optimization
		if(!use_pcm_ || other_robot_keys_for_optimization.find(symbol.key()) != other_robot_keys_for_optimization.end()){
			// Extract the 3x3 rotation matrix from the message
			Vector rotation_matrix(9);
			rotation_matrix << msg->estimate[0 + 9*i], msg->estimate[1 + 9*i], msg->estimate[2 + 9*i],
				msg->estimate[3 + 9*i], msg->estimate[4 + 9*i], msg->estimate[5 + 9*i],
				msg->estimate[6 + 9*i], msg->estimate[7 + 9*i], msg->estimate[8 + 9*i];
			// Update the optimizer with the neighbor's rotation estimate
			optimizer->updateNeighborLinearizedRotations(symbol.key(), rotation_matrix);
		} else {
			// If the key does not exist, log a warning and stop the current optimization
			RCLCPP_INFO(this->get_logger(), "Stop optimization %d. Key %c %lu doesn't exist.", robot_id, symbol.chr(), symbol.index());
			abortOptimization(false);
		}
	}

	// update neighbor flags
	// If we are still in the rotation estimation phase, update neighbor flags and progress tracking
	if(optimizer_state == OptimizerState::RotationEstimation){
		// used only with flagged initialization
		// Mark the neighboring robot as initialized (if flagged initialization is used)
		optimizer->updateNeighboringRobotInitialized(char(id + 'a'), msg->initialized);
		// Mark whether the neighboring robot has completed its rotation estimation
		neighbors_rotation_estimate_finished[id] = msg->estimation_done;
	}
	// Log progress of rotation estimation
	RCLCPP_INFO(this->get_logger(), 
		"neighborRotationHandler<%d> from robot %d done? %d (%lu/%lu)", 
		robot_id, id, msg->estimation_done, 
		optimizer->getNeighboringRobotsInit().size(), 
		optimization_order.size() - 1);

	// perform rotation optimization
	// all other robots is finished rotation optimization
	// Check if all neighbors have completed their initialization (ready to proceed with rotation estimation)
	if(optimizer->getNeighboringRobotsInit().size() == optimization_order.size() - 1){
		if(!estimation_done){
			try{
				// Perform the rotation estimation using the optimizer
				optimizer->estimateRotation();			// Estimate the rotation
				optimizer->updateRotation();			// Apply the updated rotation
				optimizer->updateInitialized(true);		// Mark the optimizer as initialized
				current_rotation_estimate_iteration++;	// Increment the iteration counter
			} catch(const std::exception& ex){
				// Log any errors encountered during rotation estimation and abort optimization
				RCLCPP_ERROR(this->get_logger(), "Stopping rotation optimization %d: %s.", robot_id, ex.what());
				abortOptimization(true);
			}

			// if change is small enough, end roation optimization
			if((optimizer->latestChange() <= rotation_estimate_change_threshold_) || (current_rotation_estimate_iteration >= optimization_maximum_iteration_)){
				rotation_estimate_finished = true;	// Mark rotation estimation as complete
				estimation_done = true;				// Indicate that the estimation is done
			}
			RCLCPP_INFO(this->get_logger(), "Rotation estimation %d iter: [%d/%d] change:%.4f", robot_id, current_rotation_estimate_iteration, optimization_maximum_iteration_, optimizer->latestChange());
		}

		// check neigbors rotation optimization state
		bool send_flag = estimation_done;
		for(int i = 0; i < optimization_order.size(); i++)
		{
			int other_robot = optimization_order[i];
			if(!neighbors_rotation_estimate_finished[other_robot] && other_robot != robot_id)
			{
				send_flag = false;	// If any neighbor is not done, do not proceed to send the estimate
			}
		}

		// Send the rotation estimate to the next robot in the optimization order
		if(!send_flag){
			// clear buffer
			for(const auto& neighbor : neighbors_within_communication_range){
				robots[neighbor].estimate_msg.pose_id.clear();
				robots[neighbor].estimate_msg.estimate.clear();
			}
			// extract rotation estimate for each loop closure
			for(const std::pair<Symbol, Symbol>& separator_symbols: optimizer->separatorsSymbols()){
				//robot_id
				int other_robot = (int)(separator_symbols.first.chr() - 'a');
				// pose id
				robots[other_robot].estimate_msg.pose_id.push_back(separator_symbols.second.index());
				// rotation estimates
				Vector rotation_estimate = optimizer->linearizedRotationAt(separator_symbols.second.key());
				for(int it = 0; it < 9; it++)
				{
					robots[other_robot].estimate_msg.estimate.push_back(rotation_estimate[it]);
				}
			}

			// send rotation estimate to the next robot in the optimization order
			int publish_flag = false;
			for(int i=0; i<optimization_order.size(); i++){
				int other_robot = optimization_order[i];
				if(other_robot == robot_id){
					publish_flag = true;	// Start publishing after reaching this robot in the order
					continue;
				}
				if(publish_flag){
					// Populate and publish the estimate message for each robot
					robots[other_robot].estimate_msg.initialized = optimizer->isRobotInitialized();
					robots[other_robot].estimate_msg.receiver_id = other_robot;
					robots[other_robot].estimate_msg.estimation_done = estimation_done;
					robots[robot_id].pub_neighbor_rotation_estimates->publish(robots[other_robot].estimate_msg);
				}
			}
			// Reset the rotation estimate state and clear initialization flags
			rotation_estimate_start = false;
			optimizer->clearNeighboringRobotInit();
		}

		// Publish the current rotation optimization state to notify neighbors
		state_msg.data = estimation_done? 1:0;
		robots[robot_id].pub_rotation_estimate_state->publish(state_msg);
	}
}

/**
 * @brief Handles incoming pose estimates from neighboring robots and updates the local pose estimation process.
 *
 * This function is part of a distributed multi-robot mapping system and is called whenever a neighboring robot
 * sends a pose estimate message. It updates the local robot's pose estimates using the information received 
 * from its neighbors, checks for convergence, and shares updates with other robots if necessary.
 *
 * Key Steps:
 * - Validate and process the incoming pose estimate message.
 * - Update the optimizer with the linearized pose estimates from neighbors.
 * - Check for convergence conditions and finalize pose estimation if needed.
 * - Share the updated pose estimates with the next robot in the optimization order.
 * - Publish the pose estimation state to inform neighboring robots about the progress.
 *
 * @param msg The incoming message containing pose estimates from a neighboring robot.
 * @param id The ID of the neighboring robot that sent the message.
 * @param robot_id The ID of the current robot
 */
void distributedMapping::neighborPoseHandler(
	const multi_agent_mapping::msg::NeighborEstimate::SharedPtr& msg,
	int id)
{
	// Ignore the message if it is not intended for this robot or if the robot is not in the pose estimation phase
	if(msg->receiver_id != robot_id || optimizer_state != OptimizerState::PoseEstimation){
		return;
	}
	// If the neighbor has completed its estimation, save the anchor offset for this neighbor
	if(msg->estimation_done){
		Point3 offset(msg->anchor_offset[0], msg->anchor_offset[1], msg->anchor_offset[2]);
		neighbors_anchor_offset[id] = offset;
	}
	// update neighbor pose estimates
	for(int i = 0; i < msg->pose_id.size(); i++){
		// Only update the estimates if the neighbor is within communication range
		if(neighbors_within_communication_range.find(id) != neighbors_within_communication_range.end())
		{
			Symbol symbol((id + 'a'), msg->pose_id[i]);
			Vector pose_vector(6);
			pose_vector << msg->estimate[0 + 6*i], msg->estimate[1 + 6*i], msg->estimate[2 + 6*i],
				msg->estimate[3 + 6*i], msg->estimate[4 + 6*i], msg->estimate[5 + 6*i];

			// Update the optimizer with the linearized pose for the given symbol (neighboring pose)
			optimizer->updateNeighborLinearizedPoses(symbol.key(), pose_vector);
		}
	}
	// update neighbor flags
	if(optimizer_state == OptimizerState::PoseEstimation){
		// used only with flagged initialization
		optimizer->updateNeighboringRobotInitialized(char(id + 'a'), msg->initialized);
		// Mark whether the neighboring robot has completed pose estimation
		neighbors_estimation_done[id] = msg->estimation_done;
		neighbors_pose_estimate_finished[id] = msg->estimation_done;
	}
	RCLCPP_INFO(
		this->get_logger(), 
		"neighborPoseHandler<%d> from robot %d, done? %d [%lu/%lu]", 
		robot_id, id, msg->estimation_done, 
		optimizer->getNeighboringRobotsInit().size(), 
		optimization_order.size() - 1
	);

	// perform pose estimation
	if(optimizer->getNeighboringRobotsInit().size() == optimization_order.size() - 1){
		if(!estimation_done){
			// pose estimation
			try{
				optimizer->estimatePoses();
				optimizer->updatePoses();
				optimizer->updateInitialized(true);
				current_pose_estimate_iteration++;
			} catch(const std::exception& ex){
				RCLCPP_ERROR(this->get_logger(), "Stopping pose estimation %d: %s.", robot_id, ex.what());
				abortOptimization(true);
			}

			// if change is small enough, end pose optimization
			if((current_pose_estimate_iteration >= optimization_maximum_iteration_) || (optimizer->latestChange() <= pose_estimate_change_threshold_)){
				pose_estimate_finished = true;
				estimation_done = true;
			}
			RCLCPP_INFO(
				this->get_logger(), 
				"--->Pose estimation<%d> iter: [%d/%d] change: %.4f.", 
				robot_id, 
				current_pose_estimate_iteration, 
				optimization_maximum_iteration_, 
				optimizer->latestChange()
			);

			// extract anchor offset
			Key first_key = KeyVector(initial_values->keys()).at(0);
			anchor_point = initial_values->at<Pose3>(first_key).translation();
			anchor_offset = anchor_point - (optimizer->currentEstimate().at<Pose3>(first_key).translation() + Point3(optimizer->linearizedPoses().at(first_key).tail(3)));

			// check neigbors pose optimization state
			bool send_flag = estimation_done;
			for(int i = 0; i < optimization_order.size(); i++){
				int other_robot = optimization_order[i];
				if(!neighbors_pose_estimate_finished[other_robot] && other_robot != robot_id)
				{
					send_flag = false;
				}
			}

			// Send the pose estimate to the next robot in the optimization order
			if(!send_flag){
				// clear buffer
				for(const auto& neighbor : neighbors_within_communication_range){
					robots[neighbor].estimate_msg.pose_id.clear();
					robots[neighbor].estimate_msg.estimate.clear();
					robots[neighbor].estimate_msg.anchor_offset.clear();
				}
			}
			// extract pose estimate from each loop closure
			for(const std::pair<Symbol, Symbol>& separator_symbols: optimizer->separatorsSymbols()){
				int other_robot = (int)(separator_symbols.first.chr() - 'a');

				robots[other_robot].estimate_msg.pose_id.push_back(separator_symbols.second.index());

				Vector pose_estimate = optimizer->linearizedPosesAt(separator_symbols.second.key());
				for(int it = 0; it < 6; it++)
				{
					robots[other_robot].estimate_msg.estimate.push_back(pose_estimate[it]);
				}
			}

			// send pose estimate
			bool publish_flag = false;
			for(int i = 0; i < optimization_order.size(); i++){
				int other_robot = optimization_order[i];
				if(other_robot == robot_id)
				{
					publish_flag = true;
					continue;
				}

				if(publish_flag)
				{
					for(int i = 0; i < 3; i++)
					{
						robots[other_robot].estimate_msg.anchor_offset.push_back(anchor_offset(i));
					}
					robots[other_robot].estimate_msg.initialized = optimizer->isRobotInitialized();
					robots[other_robot].estimate_msg.receiver_id = other_robot;
					robots[other_robot].estimate_msg.estimation_done = estimation_done;
					robots[robot_id].pub_neighbor_pose_estimates->publish(robots[other_robot].estimate_msg);
				}
			}
			// send pose optimization state
			pose_estimate_start = false;
			optimizer->clearNeighboringRobotInit();
		}
		// send optimization state
		state_msg.data = estimation_done? 1:0;
		robots[robot_id].pub_pose_estimate_state->publish(state_msg);
	}
}

/**
 * @brief Function to save current frame if its difference is more than the threshold
 * @param surroundingkeyframeAddingDistThreshold = 0.2radian
 * @param surroundingkeyframeAddingAngleThreshold = 1.0m
 */
bool distributedMapping::saveFrame(
	const Pose3& pose_to
){
	if(keyposes_cloud_3d->empty()){
		return true;
	}

	auto last_keypose = pclPointTogtsamPose3(keyposes_cloud_6d->back());
	auto pose_increment = last_keypose.inverse() * pose_to;

	float x = pose_increment.translation().x();
	float y = pose_increment.translation().y();
	float z = pose_increment.translation().z();
	float roll = pose_increment.rotation().roll();
	float pitch = pose_increment.rotation().pitch();
	float yaw = pose_increment.rotation().yaw();

	// select keyframe
	if(abs(roll) < surroundingkeyframeAddingDistThreshold && abs(pitch) < surroundingkeyframeAddingDistThreshold && abs(yaw) < surroundingkeyframeAddingDistThreshold && 
			sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingAngleThreshold){
				return false;
			}
	
	return true;
}

void distributedMapping::processKeyframesAndPose(
	const Pose3& pose_to,
	const pcl::PointCloud<PointPose3D>::Ptr frame_to,
	const rclcpp::Time& timestamp
){
	// save keyframe cloud
	pcl::copyPointCloud(*frame_to, *robots[robot_id].keyframe_cloud);
	robots[robot_id].keyframe_cloud_array.push_back(*robots[robot_id].keyframe_cloud);
	// save timestamp
	robots[robot_id].time_cloud_input_stamp = timestamp;
	robots[robot_id].time_cloud_input = timestamp.seconds();

	// add prio factor if no prior poses
	Pose3 new_pose_to;
	int poses_number = initial_values->size();
	Symbol current_symbol = Symbol('a' + robot_id, poses_number);
	if(poses_number == 0){
		RCLCPP_INFO(this->get_logger(), "Adding prior factor!!");
		// save prior pose value
		robots[robot_id].prior_odom = pose_to;
		// add prior factor to graph
		auto prior_factor = PriorFactor<Pose3>(current_symbol, pose_to, prior_noise);
		local_pose_graph_no_filtering->add(prior_factor);
		isam2_graph.add(prior_factor);

		// add prior values
		initial_values->insert(current_symbol, pose_to);
		isam2_initial_values.insert(current_symbol, pose_to);
		new_pose_to = pose_to;

		RCLCPP_INFO(
			this->get_logger(), 
			"createPrior: [%d] Translation: x = %.3f, y = %.3f, z = %.3f, Rotation: roll = %.3f, pitch = %.3f, yaw = %.3f.", 
			robot_id, 
			new_pose_to.translation().x(), 
			new_pose_to.translation().y(), 
			new_pose_to.translation().z(), 
			new_pose_to.rotation().roll(), 
			new_pose_to.rotation().pitch(), 
			new_pose_to.rotation().yaw()
		);
	} else {
		// add odometry factor if poses not empty
		RCLCPP_INFO(this->get_logger(), "Adding odom factor!!");

		// incremental odom data in local frame
		auto pose_from = pclPointTogtsamPose3(keyposes_cloud_6d->points[poses_number - 1]);
		auto pose_increment = pose_from.between(pose_to);
		Symbol previous_symbol = Symbol('a' + robot_id, poses_number - 1);
		Matrix covariance = odometry_noise->covariance();

		// add odometry factor to graph
		NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(
			previous_symbol, current_symbol, pose_increment, odometry_noise
		));
		local_pose_graph->add(factor);
		local_pose_graph_no_filtering->add(factor);
		isam2_graph.add(factor);

		// add odometry value
		isam2_initial_values.insert(current_symbol, pose_to);
		// incremental odom in global grpah frame
		pose_from = initial_values->at<Pose3>(previous_symbol);
		new_pose_to = pose_from*pose_increment;
		initial_values->insert(current_symbol, new_pose_to);

		// save factor in local map (for PCM)
		auto new_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);
		robot_local_map.addTransform(*new_factor, covariance);

		RCLCPP_INFO(
			this->get_logger(), 
			"createOdom: [%d] [%d-%d] -- [From: x = %.3f, y = %.3f, z = %.3f, roll = %.3f, pitch = %.3f, yaw = %.3f], "
			"[To: x = %.3f, y = %.3f, z = %.3f, roll = %.3f, pitch = %.3f, yaw = %.3f].", 
			robot_id, 
			poses_number - 1, 
			poses_number, 
			pose_from.translation().x(), 
			pose_from.translation().y(), 
			pose_from.translation().z(), 
			pose_from.rotation().roll(), 
			pose_from.rotation().pitch(), 
			pose_from.rotation().yaw(), 
			new_pose_to.translation().x(), 
			new_pose_to.translation().y(), 
			new_pose_to.translation().z(), 
			new_pose_to.rotation().roll(), 
			new_pose_to.rotation().pitch(), 
			new_pose_to.rotation().yaw()
		);
	}

	//optimizing
	isam2_graph.print("GTSAM Graph:\n");
	isam2_graph.resize(0);
	isam2_initial_values.clear();
	isam2_current_estimates = isam2->calculateEstimate();
	isam2_keypose_estimate = isam2_current_estimates.at<Pose3>(current_symbol);

	// save pose in local frame
	static PointPose3D pose_3d;
	pose_3d.x = isam2_keypose_estimate.translation().x();
	pose_3d.y = isam2_keypose_estimate.translation().y();
	pose_3d.z = isam2_keypose_estimate.translation().z();
	pose_3d.intensity = poses_number; // keyframe index
	keyposes_cloud_3d->push_back(pose_3d);

	static PointPose6D pose_6d;
	pose_6d.x = pose_3d.x;
	pose_6d.y = pose_3d.y;
	pose_6d.z = pose_3d.z;
	pose_6d.intensity = pose_3d.intensity;
	pose_6d.roll = isam2_keypose_estimate.rotation().roll();
	pose_6d.pitch = isam2_keypose_estimate.rotation().pitch();
	pose_6d.yaw = isam2_keypose_estimate.rotation().yaw();
	pose_6d.time = robots[robot_id].time_cloud_input; // keyframe timestamp
	keyposes_cloud_6d->push_back(pose_6d);

	RCLCPP_INFO(
		this->get_logger(), 
		"save: [%d] -- [%d] -- Translation: x = %.3f, y = %.3f, z = %.3f, Rotation: roll = %.3f, pitch = %.3f, yaw = %.3f.", 
		robot_id, 
		poses_number, 
		isam2_keypose_estimate.translation().x(), 
		isam2_keypose_estimate.translation().y(), 
		isam2_keypose_estimate.translation().z(), 
		isam2_keypose_estimate.rotation().roll(), 
		isam2_keypose_estimate.rotation().pitch(), 
		isam2_keypose_estimate.rotation().yaw()
	);

	// save path for viz
	updateLocalPath(pose_6d);
	updateGlobalPath(new_pose_to);
}

void distributedMapping::updateLocalPath(
	const PointPose6D& pose
){
	static geometry_msgs::msg::PoseStamped pose_stamped_msg;
	pose_stamped_msg.header.stamp = this->get_clock()->now();
	pose_stamped_msg.header.frame_id = world_frame_;
	pose_stamped_msg.pose.position.x = pose.x;
	pose_stamped_msg.pose.position.y = pose.y;
	pose_stamped_msg.pose.position.z = pose.z;
	tf2::Quaternion q;
    q.setRPY(pose.roll, pose.pitch, pose.yaw);
    pose_stamped_msg.pose.orientation = tf2::toMsg(q);

    // Add the pose to thje local path
    local_path.poses.push_back(pose_stamped_msg);
};

void distributedMapping::updateGlobalPath(
	const Pose3& pose
){
	static geometry_msgs::msg::PoseStamped pose_stamped_msg;
	pose_stamped_msg.header.stamp = this->get_clock()->now();
	pose_stamped_msg.header.frame_id = world_frame_;
	pose_stamped_msg.pose.position.x = pose.translation().x();
	pose_stamped_msg.pose.position.y = pose.translation().y();
	pose_stamped_msg.pose.position.z = pose.translation().z();
	pose_stamped_msg.pose.orientation.x = pose.rotation().toQuaternion().x();
	pose_stamped_msg.pose.orientation.y = pose.rotation().toQuaternion().y();
	pose_stamped_msg.pose.orientation.z = pose.rotation().toQuaternion().z();
	pose_stamped_msg.pose.orientation.w = pose.rotation().toQuaternion().w();

	global_path.poses.push_back(pose_stamped_msg);
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
bool distributedMapping::updatePoses(){

    RCLCPP_INFO(this->get_logger(), "[DistributedMapping - mapOptimization] -> checking if intra robot loop closure has occured for [%d]", robot_id);

	bool return_value = false;

	if(keyposes_cloud_3d->empty()){
		return return_value;
	}
	if(intra_robot_loop_close_flag){
		// clear path
		local_path.poses.clear();

		// add estimates
		for(const Values::ConstKeyValuePair &key_value: isam2_current_estimates){
			//update key poses
			Symbol key = key_value.key;
			int index = key.index();
			Pose3 pose = isam2_current_estimates.at<Pose3>(key);

			keyposes_cloud_3d->points[index].x = pose.translation().x();
			keyposes_cloud_3d->points[index].y = pose.translation().y();
			keyposes_cloud_3d->points[index].z = pose.translation().z();

			keyposes_cloud_6d->points[index].x = keyposes_cloud_3d->points[index].x;
			keyposes_cloud_6d->points[index].y = keyposes_cloud_3d->points[index].y;
			keyposes_cloud_6d->points[index].z = keyposes_cloud_3d->points[index].z;
			keyposes_cloud_6d->points[index].roll = pose.rotation().roll();
			keyposes_cloud_6d->points[index].pitch = pose.rotation().pitch();
			keyposes_cloud_6d->points[index].yaw = pose.rotation().yaw();
			

			updateLocalPath(keyposes_cloud_6d->points[index]);
		}
		// Reset the intra-robot loop closre flag and set return value to true
        intra_robot_loop_close_flag = false;
        return_value = true;
	}
	// copy the updated keyposes
    *copy_keyposes_cloud_3d = *keyposes_cloud_3d;
    *copy_keyposes_cloud_6d = *keyposes_cloud_6d;

    // update the most recent keypose, representing robots current position
    isam2_keypose_estimate = pclPointTogtsamPose3(keyposes_cloud_6d->back());
    return return_value;
}

void distributedMapping::makeIrisDescriptor(){
	while(!store_descriptors.empty()){
		auto msg_data = store_descriptors.front().second;
		auto msg_id = store_descriptors.front().first;
		store_descriptors.pop_front();
		keyframe_descriptor->saveDescriptorAndKey(msg_data.values.data(), msg_id, msg_data.index); // default using LiDAR IRIS Descriptor
	}
	if(initial_values->empty() || (!intra_robot_loop_closure_enable_ && !inter_robot_loop_closure_enable_)){
		return;
	}

	// downsample keyframe
	cloud_for_descript_ds->clear();
	downsample_filter_for_descriptor.setInputCloud(robots[robot_id].keyframe_cloud);
	downsample_filter_for_descriptor.filter(*cloud_for_descript_ds);

	auto descriptor_vec = keyframe_descriptor->makeAndSaveDescriptorAndKey(*cloud_for_descript_ds, robot_id, initial_values->size()-1); // default using LiDAR IRIS Descriptor
	RCLCPP_INFO(this->get_logger(), "[DistributedMapping - mapOptimization] -> Finished makeIrisDescriptors[%d].", robot_id);

	// extract descriptors values
	global_descriptor_msg.values.swap(descriptor_vec);
	// keyfame index
	global_descriptor_msg.index = initial_values->size()-1;
	global_descriptor_msg.header.stamp = robots[robot_id].time_cloud_input_stamp;
	// publish message
	robots[robot_id].pub_descriptors->publish(global_descriptor_msg);
}