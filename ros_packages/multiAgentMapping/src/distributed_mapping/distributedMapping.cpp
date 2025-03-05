#include "multiAgentMapping/distributed_mapping/distributedMapping.hpp"

void distributedMapping::globalDescriptorHandler(
	const multi_agent_mapping::msg::GlobalDescriptor::SharedPtr& msg,
	int id){

	RCLCPP_INFO(
        this->get_logger(), 
        "[CALLBACK INITIATED] [globalDescriptorHandler(%d)] Received Global Descriptor.", 
        id
    );
	// save timestamp
    robots[id].time_cloud_input_stamp = msg->header.stamp;
    robots[id].time_cloud_input = rclcpp::Time(robots[id].time_cloud_input_stamp).seconds();
	// Log message details of saved descriptors
    RCLCPP_INFO(
        this->get_logger(), 
        "[globalDescriptorHandler(%d)] Saving Descriptor. Index: %d | First Value: %f", 
        id, msg->index, msg->values.empty() ? -1.0 : msg->values[0]
    );
	store_descriptors.emplace_back(make_pair(id,*msg));
}

void distributedMapping::optStateHandler(
	const std_msgs::msg::Int8::SharedPtr& msg,
	int id)
{
	// Validate robot ID
	if (id >= number_of_robots_ || id < 0) {
		RCLCPP_ERROR(this->get_logger(), 
			"[optStateHandler] Invalid robot ID: %d", id);
		return;
	}

	neighbors_started_optimization[id] = (OptimizerState)msg->data <= OptimizerState::Start;
	neighbor_state[id] = (OptimizerState)msg->data;
	neighbors_lowest_id_included[id] = lowest_id_included;
	
	// Safe insertion into communication range set
	if (neighbors_within_communication_range.find(id) == neighbors_within_communication_range.end()) {
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
	for(int i = 0; i < msg->pose_id.size(); i++){
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
				RCLCPP_ERROR(this->get_logger(), "[neighborRotationHandler] - Stopping rotation optimization %d: %s.", robot_id, ex.what());
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
			for(const auto& neighbor : neighbors_within_communication_range) {
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
				RCLCPP_ERROR(this->get_logger(), "[neighborPoseHandler] - Stopping pose estimation %d: %s.", robot_id, ex.what());
				std::cerr << "[neighborPoseHandler] - Stopping pose estimation " << robot_id << ": " << ex.what() << std::endl;
				abortOptimization(true);
			}

			// if change is small enough, end pose optimization
			if((current_pose_estimate_iteration >= optimization_maximum_iteration_) || (optimizer->latestChange() <= pose_estimate_change_threshold_)){
				pose_estimate_finished = true;
				estimation_done = true;
			}
			RCLCPP_INFO(
				this->get_logger(), 
				"[neighborPoseHandler] - --->Pose estimation<%d> iter: [%d/%d] change: %.4f.", 
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
				for(const auto& neighbor : neighbors_within_communication_range) {
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
					for(int i = 0; i < 3; i++) {
						robots[other_robot].estimate_msg.anchor_offset.push_back(anchor_offset[i]);
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

	RCLCPP_INFO(this->get_logger(), "[distributedMapping::processKeyframesAndPose] -> Accessing robot_id: %d", robot_id);

	// save keyframe cloud
	pcl::copyPointCloud(*frame_to, *robots[robot_id].keyframe_cloud);
	robots[robot_id].keyframe_cloud_array.push_back(*robots[robot_id].keyframe_cloud);
	// save timestamp
	robots[robot_id].time_cloud_input_stamp = timestamp;
	robots[robot_id].time_cloud_input = timestamp.seconds();

	// add prior factor if no prior poses
	Pose3 new_pose_to;
	int poses_number = initial_values->size();
	RCLCPP_INFO(this->get_logger(), "[distributedMapping::processKeyframesAndPose] -> Current poses_number: %d", poses_number);
	Symbol current_symbol = Symbol('a' + robot_id, poses_number);
	if(poses_number == 0){
		RCLCPP_INFO(this->get_logger(), "[distributedMapping::processKeyframesAndPose] -> Adding prior factor!!");
		// save piror pose value
		robots[robot_id].piror_odom = pose_to;
		// add piror factor to graph
		auto prior_factor = PriorFactor<Pose3>(current_symbol, pose_to, prior_noise);
		local_pose_graph_no_filtering->add(prior_factor);
		isam2_graph.add(prior_factor);

		// add prior values
		initial_values->insert(current_symbol, pose_to);
		isam2_initial_values.insert(current_symbol, pose_to);
		new_pose_to = pose_to;

		RCLCPP_INFO(
			this->get_logger(), 
			"[distributedMapping::processKeyframesAndPose] -> createPrior: [%d] Translation: x = %.3f, y = %.3f, z = %.3f, Rotation: roll = %.3f, pitch = %.3f, yaw = %.3f.", 
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
		RCLCPP_INFO(this->get_logger(), "[distributedMapping::processKeyframesAndPose] -> Adding odom factor!!");

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
			"[distributedMapping::processKeyframesAndPose] -> createOdom: [%d] [%d-%d] -- [From: x = %.3f, y = %.3f, z = %.3f, roll = %.3f, pitch = %.3f, yaw = %.3f], "
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
	isam2_graph.print("[distributedMapping::processKeyframesAndPose] -> GTSAM Graph:\n");
	isam2->update(isam2_graph, isam2_initial_values);
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
	RCLCPP_INFO(this->get_logger(), "[distributedMapping::processKeyframesAndPose] -> Saved pose in Local Frame");

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
	RCLCPP_INFO(this->get_logger(), "[distributedMapping::processKeyframesAndPose] -> Saved pose in Global Frame");

	RCLCPP_INFO(
		this->get_logger(), 
		"[distributedMapping::processKeyframesAndPose] -> save: robot_id: [%d] & pose_number:  [%d] -- Translation: x = %.3f, y = %.3f, z = %.3f, Rotation: roll = %.3f, pitch = %.3f, yaw = %.3f.", 
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

/**
 * @brief Processes and generates IRIS descriptors for LiDAR keyframes
 * 
 * This function handles two main tasks:
 * 1. Processes stored descriptors from other robots in the queue
 * 2. Generates and publishes new descriptors for the current robot's keyframe
 * 
 * The function will:
 * - Process any pending descriptors from other robots
 * - Generate a new descriptor for the current keyframe if loop closure is enabled
 * - Downsample the current keyframe cloud
 * - Create and verify the descriptor message
 * - Publish the descriptor for other robots to use
 */
void distributedMapping::makeIrisDescriptor() {
	try{
	// Process stored descriptors from other robots
		while(!store_descriptors.empty()){
			auto msg_data = store_descriptors.front().second;
			auto msg_id = store_descriptors.front().first;
			store_descriptors.pop_front();
            keyframe_descriptor->saveDescriptorAndKey(msg_data.values.data(), msg_id, msg_data.index);
		}

		// Skip if no initial values or loop closure is disabled
		if(initial_values->empty() || (!intra_robot_loop_closure_enable_ && !inter_robot_loop_closure_enable_)){
			return;
		}

		// Downsample keyframe for descriptor generation
		cloud_for_descript_ds->clear();
		downsample_filter_for_descriptor.setInputCloud(robots[robot_id].keyframe_cloud);
		downsample_filter_for_descriptor.filter(*cloud_for_descript_ds);

		if (cloud_for_descript_ds->empty()) {
			RCLCPP_WARN(this->get_logger(), 
				"[makeIrisDescriptor] Empty downsampled cloud, skipping descriptor generation");
			return;
		}

		RCLCPP_INFO(this->get_logger(), "[DEBUG] Filtered LiDAR PointCloud Size: %lu", cloud_for_descript_ds->size());

		// Generate descriptor and prepare message
		auto descriptor_vec = keyframe_descriptor->makeAndSaveDescriptorAndKey(*cloud_for_descript_ds, robot_id, initial_values->size()-1);

        global_descriptor_msg.values.swap(descriptor_vec);
		global_descriptor_msg.index = initial_values->size()-1;
		global_descriptor_msg.header.stamp = robots[robot_id].time_cloud_input_stamp;
		robots[robot_id].pub_descriptors->publish(global_descriptor_msg);

		RCLCPP_INFO(this->get_logger(), "[DistributedMapping - mapOptimization] -> Finished makeIrisDescriptors[%d].", robot_id);
	} catch (const std::exception& e) {
		std::cerr << "[makeIrisDescriptor] Error in makeIrisDescriptor: " << e.what() << std::endl;
	}
}
void distributedMapping::publishPath(){
	// publish global path
	if(pub_global_path->get_subscription_count()!=0){
		global_path.header.stamp = robots[robot_id].time_cloud_input_stamp;
		global_path.header.frame_id = world_frame_;
		pub_global_path->publish(global_path);
	}
	// publish local path
	if(pub_local_path->get_subscription_count()!=0){
		local_path.header.stamp = robots[robot_id].time_cloud_input_stamp;
		local_path.header.frame_id = robots[robot_id].odom_frame_;
		pub_local_path->publish(local_path);
	}
}

void distributedMapping::publishTransformation(
	const rclcpp::Time& timestamp
){
	static std::shared_ptr<tf2_ros::TransformBroadcaster> world_to_odom_tf_broadcaster = 
    std::make_shared<tf2_ros::TransformBroadcaster>(this);
	static Symbol first_key((robot_id + 'a'), 0);

	Pose3 first_pose = initial_values->at<Pose3>(first_key);
	Pose3 old_first_pose = robots[robot_id].piror_odom;
	Pose3 pose_between = first_pose * old_first_pose.inverse();

	// create a quaternion using tf2
	tf2::Quaternion quaternion;
	quaternion.setRPY(
		pose_between.rotation().roll(),
		pose_between.rotation().pitch(),
		pose_between.rotation().yaw()
	);

	// Create the TransformStamped message
    geometry_msgs::msg::TransformStamped world_to_odom;
    world_to_odom.header.stamp = timestamp;
    world_to_odom.header.frame_id = world_frame_;
    world_to_odom.child_frame_id = robots[robot_id].odom_frame_;

    // Set translation
    world_to_odom.transform.translation.x = pose_between.translation().x();
    world_to_odom.transform.translation.y = pose_between.translation().y();
    world_to_odom.transform.translation.z = pose_between.translation().z();

    // Set rotation
    world_to_odom.transform.rotation.x = quaternion.x();
    world_to_odom.transform.rotation.y = quaternion.y();
    world_to_odom.transform.rotation.z = quaternion.z();
    world_to_odom.transform.rotation.w = quaternion.w();

    // Broadcast the transform
    world_to_odom_tf_broadcaster->sendTransform(world_to_odom);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
	class distributedMapping: distributed mapping
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void distributedMapping::abortOptimization(
	const bool& log_info)
{
	for(const auto& transform : robot_local_map_backup.getTransforms().transforms){
		if(robot_local_map.getTransforms().transforms.find(transform.first) == robot_local_map.getTransforms().transforms.end()){
			SharedNoiseModel model = noiseModel::Gaussian::Covariance(transform.second.pose.covariance_matrix);
			auto factor = BetweenFactor<Pose3>(transform.second.i, transform.second.j, transform.second.pose.pose, model);
			robot_local_map.addTransform(factor, transform.second.pose.covariance_matrix);
			local_pose_graph->push_back(factor);
		}
	}
	changeOptimizerState(OptimizerState::Idle);
}

bool distributedMapping::startOptimizationCondition(){
	// check if all neighbors started
	bool all_neighbor_started = !neighbors_started_optimization.empty();
	for(const auto& neighbor_started : neighbors_started_optimization)
	{
		all_neighbor_started &= neighbor_started.second && (neighbor_state[neighbor_started.first] <= OptimizerState::Start);
	}
	return all_neighbor_started && sent_start_optimization_flag;
}

void distributedMapping::updateOptimizer(){
	// load subgraphs
	graph_values_vec = std::make_pair(local_pose_graph, initial_values);
	optimizer->loadSubgraphAndCreateSubgraphEdge(graph_values_vec);

	// add prior to the first robot
	std::pair<int, int> robot_pair = std::make_pair(robot_id, lowest_id_included);
	for(const auto& neighbor_lowest_id : neighbors_lowest_id_included){
		if(neighbors_within_communication_range.find(neighbor_lowest_id.first) != neighbors_within_communication_range.end()){
			if(neighbor_lowest_id.second < robot_pair.second){
				robot_pair = neighbor_lowest_id;
				if(lowest_id_to_included > neighbor_lowest_id.second){
					lowest_id_to_included = neighbor_lowest_id.second;
				}
			}
			if(neighbor_lowest_id.second == robot_pair.second){
				if(robot_pair.first > neighbor_lowest_id.first){
					robot_pair = neighbor_lowest_id;
				}
			}
		}
	}

	// first robot always have the prior
	prior_owner = robot_pair.first;
	RCLCPP_INFO(this->get_logger(), "priorOwner<%d> %d", robot_id, prior_owner);
	if(robot_pair.first == robot_id){
		static Key prior_key = Symbol('a' + robot_id, 0);
		optimizer->addPrior(prior_key, robots[robot_id].piror_odom, prior_noise);
		prior_added = true;
	}

	// check for graph connectivity
	RCLCPP_INFO(this->get_logger(), "Checking graph connectivity <%d>", robot_id);
	neighboring_robots = optimizer->getNeighboringChars();
	if(neighboring_robots.size() > 0){
		graph_disconnected = false;
	}
	bool has_seperator_with_neighbor = false;
	// check neighbor within communication range
	for(const auto& neighbor : neighbors_within_communication_range){
		if(neighboring_robots.find((char)(neighbor + 'a')) != neighboring_robots.end()){
			has_seperator_with_neighbor = true;
			neighbors_estimation_done[neighbor] = false;
			neighbors_pose_estimate_finished[neighbor] = false;
			neighbors_rotation_estimate_finished[neighbor] = false;
		}
	}
	if(!has_seperator_with_neighbor){
		changeOptimizerState(OptimizerState::Idle);
		RCLCPP_INFO(this->get_logger(), "Changed Optimizer State <%d>", robot_id);
	}
}

void distributedMapping::outliersFiltering(){
	robot_local_map_backup = robot_local_map;
	if(use_pcm_)
	{
		measurements_accepted_num = 0;
		measurements_rejected_num = 0;

		RCLCPP_INFO(this->get_logger(), 
			"[outliersFiltering] Processing %zu neighbors within communication range", 
			neighbors_within_communication_range.size());

		// perform pairwise consistency maximization for each pair robot 
		for(const auto& neighbor : neighbors_within_communication_range){
			if(neighbor == robot_id || pose_estimates_from_neighbors.find(neighbor) == pose_estimates_from_neighbors.end()){
				continue;
			}

			// get other robot trajectory
			graph_utils::Transforms empty_transforms;
			auto neighbor_local_info = robot_measurements::RobotLocalMap(
				pose_estimates_from_neighbors.at(neighbor), empty_transforms, robot_local_map.getLoopClosures());

			// get inter-robot loop closure measurements
			graph_utils::Transforms loop_closures_transforms;
			for (const auto& transform : robot_local_map.getTransforms().transforms)
			{
				auto robot0 = (int) (Symbol(transform.second.i).chr()-97);
				auto robot1 = (int) (Symbol(transform.second.j).chr()-97);
				if(robot0 == neighbor || robot1 == neighbor)
				{
					loop_closures_transforms.transforms.insert(transform);
				}
			}

			RCLCPP_INFO(this->get_logger(), 
				"[outliersFiltering] Found loop closure transforms between robots %d and %d", robot_id, neighbor);

			auto inter_robot_measurements = robot_measurements::InterRobotMeasurements(
				loop_closures_transforms, optimizer->robotName(), (char)neighbor + 97);

			// local trajectory 
			auto local_info = robot_measurements::RobotLocalMap(
				pose_estimates_from_neighbors.at(robot_id), robot_local_map.getTransforms(), robot_local_map.getLoopClosures());

			RCLCPP_INFO(this->get_logger(), 
				"[outliersFiltering] Creating global map for PCM between robots %d and %d", 
				robot_id, neighbor);

			// create global map
			auto globalMap = global_map::GlobalMap(local_info, neighbor_local_info,
				inter_robot_measurements, pcm_threshold_, use_heuristics_);
			
			RCLCPP_INFO(this->get_logger(), 
				"[outliersFiltering] Created global map for PCM between robots %d and %d", 
				robot_id, neighbor);

			// Execute PCM with error handling
			std::pair<std::vector<int>, int> max_clique_info;
			try {
				max_clique_info = globalMap.pairwiseConsistencyMaximization();
			} catch (const std::exception& e) {
				RCLCPP_ERROR(this->get_logger(), 
					"[outliersFiltering] PCM failed for robots %d and %d: %s", 
					robot_id, neighbor, e.what());
				continue;
			}
			// pairwise consistency maximization
			// auto max_clique_info = globalMap.pairwiseConsistencyMaximization();
			RCLCPP_INFO(this->get_logger(), "max_clique_info");
			std::vector<int> max_clique = max_clique_info.first;
			RCLCPP_INFO(this->get_logger(), "max_clique_info.first");
			measurements_accepted_num += max_clique.size();
			RCLCPP_INFO(this->get_logger(), "max_clique_info.size");
			measurements_rejected_num += max_clique_info.second;
			RCLCPP_INFO(this->get_logger(), "max_clique_info.second");

			RCLCPP_INFO(this->get_logger(), 
				"[outliersFiltering] PCM Results - Robot %d with %d:\n"
				"  - Max clique size: %zu\n"
				"  - Accepted measurements: %d\n"
				"  - Rejected measurements: %d",
				robot_id, neighbor, max_clique.size(), 
				measurements_accepted_num, measurements_rejected_num);

			// retrieve indexes of rejected measurements
			auto loopclosures_ids = optimizer->loopclosureEdge();
			std::vector<int> rejected_loopclosure_ids;
			rejected_loopclosure_ids.reserve(1000);
			std::set<std::pair<Key, Key>> accepted_key_pairs, rejected_key_pairs;
			for(int i = 0; i < loopclosures_ids.size(); i++){
				if(distributed_pcm::DistributedPCM::isloopclosureToBeRejected(max_clique, loopclosures_ids[i],
					loop_closures_transforms, inter_robot_measurements.getLoopClosures(), optimizer)){
					rejected_loopclosure_ids.emplace_back(i);

					// Update robot local map and store keys
					auto loopclosure_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(
						optimizer->currentGraph().at(loopclosures_ids[i]));
					robot_local_map.removeTransform(std::make_pair(loopclosure_factor->keys().at(0), loopclosure_factor->keys().at(1)));
					optimizer->eraseseparatorsSymbols(std::make_pair(loopclosure_factor->keys().at(0), loopclosure_factor->keys().at(1)));
					optimizer->eraseseparatorsSymbols(std::make_pair(loopclosure_factor->keys().at(1), loopclosure_factor->keys().at(0)));
					rejected_key_pairs.insert(std::make_pair(loopclosure_factor->keys().at(0), loopclosure_factor->keys().at(1)));
				} else {
					auto loopclosure_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(
						optimizer->currentGraph().at(loopclosures_ids[i]));
					accepted_key_pairs.insert(std::make_pair(loopclosure_factor->keys().at(0), loopclosure_factor->keys().at(1)));
				}
			}

			// remove measurements not in the max clique
			RCLCPP_INFO(this->get_logger(), "remove measurements not in the max clique");
			int number_loopclosure_ids_removed = 0;
			for(const auto& index : rejected_loopclosure_ids){
				auto factor_id = loopclosures_ids[index] - number_loopclosure_ids_removed;
				number_loopclosure_ids_removed++;
				optimizer->eraseFactor(factor_id);
				local_pose_graph->erase(local_pose_graph->begin()+factor_id);
			}

			// Update loopclosure ids
			RCLCPP_INFO(this->get_logger(), "Update loopclosure ids");
			std::vector<size_t> new_loopclosure_ids;
			new_loopclosure_ids.reserve(10000);
			int number_of_edges = optimizer->currentGraph().size();
			for(int i = 0; i < number_of_edges; i++){
				auto keys = optimizer->currentGraph().at(i)->keys();
				if(keys.size() == 2){
					char robot0 = symbolChr(keys.at(0));
					char robot1 = symbolChr(keys.at(1));
					int index0 = symbolIndex(keys.at(0));
					int index1 = symbolIndex(keys.at(1));
					if(robot0 != robot1){
						new_loopclosure_ids.push_back(i);
					}
				}
			}
			optimizer->setloopclosureIds(new_loopclosure_ids);
			RCLCPP_INFO(this->get_logger(), "Update loopclosure ids done");
			// save accepted pair
			for(const auto& accepted_pair : accepted_key_pairs){
				accepted_keys.insert(accepted_pair);
				if(Symbol(accepted_pair.first).chr() == char(robot_id + 'a')){
					other_robot_keys_for_optimization.insert(accepted_pair.second);
				} else {
					other_robot_keys_for_optimization.insert(accepted_pair.first);
				}
			}
			// save rejected pair
			for(const auto& rejected_pair : rejected_key_pairs){
				rejected_keys.insert(rejected_pair);
			}

			RCLCPP_INFO(this->get_logger(), "outliersFiltering<%d>, other=%d, max clique size=%d, removed=%d", 
            robot_id, neighbor, static_cast<int>(max_clique.size()), max_clique_info.second);
		}
	}
}

/**
 * @brief Computes the order in which the robots should optimize their poses during the distributed mapping process.
 *
 * This function determines the optimization order by considering the connectivity between robots 
 * and selecting robots with the most connections (edges) to previously selected robots in the order.
 *
 * Steps:
 * 1. Initialize the order with the prior owner of the optimization.
 * 2. For each robot within the communication range (excluding already ordered robots), 
 *    count the number of connections (edges) to robots already in the optimization order.
 * 3. Select the robot with the maximum number of edges and add it to the order.
 * 4. Repeat until all robots within the communication range are ordered.
 * 5. Check if the current robot (`robot_id`) is part of the determined order and update the `in_order` flag.
 */
void distributedMapping::computeOptimizationOrder() {
    // Clear any existing optimization order
    optimization_order.clear();

    // Start the order with the prior owner of the optimization
    optimization_order.emplace_back(prior_owner);

    // Map to store the number of edges for each robot
    std::map<int, int> edges_num;

    // Robots to consider for optimization (within communication range + current robot)
    auto robots_consider = neighbors_within_communication_range;
    robots_consider.insert(robot_id);  // Ensure the current robot is included

    // Loop until no more robots can be added to the order
    while (true) {
        edges_num.clear();

        // Calculate the number of edges for each robot not yet in the optimization order
        for (const auto& robot0 : robots_consider) {
            if (std::find(optimization_order.begin(), optimization_order.end(), robot0) == optimization_order.end()) {
                int robot_edges_num = 0;

                // Count the edges connecting this robot to the robots already in the order
                for (size_t robot1 : optimization_order) {
                    robot_edges_num += adjacency_matrix(robot0, robot1);
                }

                // Only consider robots with at least one edge connecting to the current order
                if (robot_edges_num != 0) {
                    edges_num.insert(std::make_pair(robot0, robot_edges_num));
                }
            }
        }

        // If no more robots can be added, break the loop
        if (edges_num.empty()) {
            break;
        }

        // Find the robot with the maximum number of edges
        int maximum_edge_num = -1;
        int maximum_robot = -1;
        for (const auto& robot_info : edges_num) {
            if (robot_info.second > maximum_edge_num) {
                maximum_edge_num = robot_info.second;
                maximum_robot = robot_info.first;
            }
        }

        // Add the robot with the most edges to the optimization order
        optimization_order.emplace_back(maximum_robot);

        // Remove the robot from consideration in the current loop iteration
        edges_num.erase(maximum_robot);
    }

    // Determine if the current robot (`robot_id`) is included in the optimization order
    in_order = false;
    for (int i = 0; i < optimization_order.size(); i++) {
        if (optimization_order[i] == robot_id) {
            in_order = true;
        }
    }
}

void distributedMapping::initializePoseGraphOptimization(){
	// load graph&value and check graph connectivity
	updateOptimizer();

	// fliter outlier with distributed PCM method
	outliersFiltering();
	
	// ordering
	computeOptimizationOrder();

	// initialize
	optimizer->updateInitialized(false);
	optimizer->clearNeighboringRobotInit();
	rotation_estimate_finished = false;
	pose_estimate_finished = false;
	estimation_done = false;
}


bool distributedMapping::rotationEstimationStoppingBarrier(){
	// check neighbor state
	bool in_turn = true;
	neighboring_robots = optimizer->getNeighboringChars();
	for(int i = 0; i < optimization_order.size(); i++){
		if(optimization_order[i] != robot_id && neighboring_robots.find((char)(optimization_order[i] + 'a')) != neighboring_robots.end()){
			in_turn &= (neighbor_state[optimization_order[i]] == OptimizerState::RotationEstimation);
		}
	}
	// send rotation estimate to the pre-order robot
	if(in_turn && !rotation_estimate_start){
		rotation_estimate_start = true;
		// clear buffer
		for(const auto& neighbor : neighbors_within_communication_range){
			robots[neighbor].estimate_msg.pose_id.clear();
			robots[neighbor].estimate_msg.estimate.clear();
		}
		// extract rotation estimate for each loop closure
		for(const std::pair<Symbol, Symbol>& separator_symbols: optimizer->separatorsSymbols()){
			// robot id
			int other_robot = (int)(separator_symbols.first.chr() - 'a');
			// pose id
			robots[other_robot].estimate_msg.pose_id.push_back(separator_symbols.second.index());
			// rotation estimates
			Vector rotation_estimate = optimizer->linearizedRotationAt(separator_symbols.second.key());
			for(int it = 0; it < 9; it++){
				robots[other_robot].estimate_msg.estimate.push_back(rotation_estimate[it]);
			}
		}
		// send rotation estimate
		for(int i = 0; i < optimization_order.size(); i++){
			int other_robot = optimization_order[i];
			if(other_robot == robot_id){
				break;
			}

			robots[other_robot].estimate_msg.initialized = optimizer->isRobotInitialized();
			robots[other_robot].estimate_msg.receiver_id = other_robot;
			robots[other_robot].estimate_msg.estimation_done = estimation_done;
			robots[robot_id].pub_neighbor_rotation_estimates->publish(robots[other_robot].estimate_msg);
		}
	}

	// check if neighbors have begun pose estimation
	bool all_finished_rotation_estimation = true;
	for(const auto& neighbor : neighbors_within_communication_range){
		all_finished_rotation_estimation &= (neighbor_state[neighbor] == OptimizerState::PoseEstimation);
	}
	if(all_finished_rotation_estimation){
		return true;
	}

	// neighbors have finished rotation estimation
	bool stop_rotation_estimation = rotation_estimate_finished;
	for(int i = 0; i < optimization_order.size(); i++){
		int otherRobot = optimization_order[i];
		if(otherRobot != robot_id){
			stop_rotation_estimation &= neighbors_rotation_estimate_finished[otherRobot] || 
				neighbor_state[otherRobot] > optimizer_state;
		}
	}
	return stop_rotation_estimation;
}

void distributedMapping::removeInactiveNeighbors(){
	std::vector<int> removed_neighbors;
	removed_neighbors.reserve(1000);
	for(const auto& neighbor : neighbors_within_communication_range){
		if(neighbor_state[neighbor] == OptimizerState::Idle){
			removed_neighbors.emplace_back(neighbor);
		}
	}
	for(const auto& neighbor : removed_neighbors){
		neighbors_within_communication_range.erase(neighbor);;
		neighbors_rotation_estimate_finished.erase(neighbor);
		neighbors_pose_estimate_finished.erase(neighbor);
		neighbors_estimation_done.erase(neighbor);
		neighbors_lowest_id_included.erase(neighbor);
	}
	if(neighbors_within_communication_range.empty()){
		RCLCPP_INFO(this->get_logger(), "Stop optimization<%d>, there are inactive neighbors.",robot_id);
		abortOptimization(false);
	}
}

void distributedMapping::failSafeCheck(){
	if(latest_change == optimizer->latestChange()){
		steps_without_change++;
	} else {
		steps_without_change = 0;
		latest_change = optimizer->latestChange();
	}

	// wait enough time to receive data from neighbors
	if(steps_without_change > fail_safe_steps_ * neighbors_within_communication_range.size()){
		RCLCPP_INFO(this->get_logger(), "No progress<%d>, Stop optimization", robot_id);
		abortOptimization(false);
	}
}

void distributedMapping::initializePoseEstimation(){
	optimizer->convertLinearizedRotationToPoses();
	Values neighbors = optimizer->neighbors();
	for(const Values::ConstKeyValuePair& key_value: neighbors){
		Key key = key_value.key;
		// pick linear rotation estimate
		VectorValues neighbor_estimate_rot_lin;
		neighbor_estimate_rot_lin.insert(key, optimizer->neighborsLinearizedRotationsAt(key));
		// make a pose out of it
		Values neighbor_rot_estimate = 
			InitializePose3::normalizeRelaxedRotations(neighbor_estimate_rot_lin);
		Values neighbor_pose_estimate = 
			distributed_mapper::evaluation_utils::pose3WithZeroTranslation(neighbor_rot_estimate);
		// store it
		optimizer->updateNeighbor(key, neighbor_pose_estimate.at<Pose3>(key));
	}

	// reset flags for flagged initialization.
	optimizer->updateInitialized(false);
	optimizer->clearNeighboringRobotInit();
	estimation_done = false;
	for(auto& neighbor_done : neighbors_estimation_done){
		neighbor_done.second = false;
	}
	optimizer->resetLatestChange();
}

bool distributedMapping::poseEstimationStoppingBarrier(){
	// check neighbor state
	bool in_turn = true;
	// neighboring_robots = optimizer->getNeighboringChars();
	for(int i = 0; i < optimization_order.size(); i++){
		if(optimization_order[i] != robot_id && neighbors_within_communication_range.find(optimization_order[i]) != neighbors_within_communication_range.end()){
			in_turn &= (neighbor_state[optimization_order[i]] == OptimizerState::PoseEstimation);
		}
	}
	// send pose estimate to the pre-order robot
	if(in_turn && !pose_estimate_start){
		pose_estimate_start = true;
		// clear buffer
		for(const auto& neighbor : neighbors_within_communication_range) {
			robots[neighbor].estimate_msg.pose_id.clear();
			robots[neighbor].estimate_msg.estimate.clear();
			robots[neighbor].estimate_msg.anchor_offset.clear();
		}
		// extract pose estimate from each loop closure
		for(const std::pair<Symbol, Symbol>& separator_symbols: optimizer->separatorsSymbols()) {
			int other_robot = (int)(separator_symbols.first.chr() - 'a');
			
			if (other_robot >= robots.size()) {
				RCLCPP_ERROR(this->get_logger(), "[poseEstimationStoppingBarrier] Invalid robot index: %d", other_robot);
				continue;
			}

			robots[other_robot].estimate_msg.pose_id.push_back(separator_symbols.second.index());

			Vector pose_estimate = optimizer->linearizedPosesAt(separator_symbols.second.key());
			for(int it = 0; it < 6; it++) {
				robots[other_robot].estimate_msg.estimate.push_back(pose_estimate[it]);
			}
		}
		// send pose estimate
		for(int i = 0; i < optimization_order.size(); i++){
			int other_robot = optimization_order[i];
			if(other_robot == robot_id){
				break;
			}
			std::vector<double> anchor_vector = {
				anchor_offset.x(),
				anchor_offset.y(),
				anchor_offset.z()
			};
			for (int i = 0; i < 3; i++) {
				robots[other_robot].estimate_msg.anchor_offset.push_back(anchor_vector[i]);
			}
			robots[other_robot].estimate_msg.initialized = optimizer->isRobotInitialized();
			robots[other_robot].estimate_msg.receiver_id = other_robot;
			robots[other_robot].estimate_msg.estimation_done = pose_estimate_finished;
			robots[robot_id].pub_neighbor_pose_estimates->publish(robots[other_robot].estimate_msg);

		}
	}

	// check if others have ended optimization
	bool all_finished_pose_estimation = true;
	for(const auto& neighbor : neighbors_within_communication_range){
		bool other_robot_finished = (neighbor_state[neighbor] != OptimizerState::PoseEstimation) &&
			(neighbor_state[neighbor] != OptimizerState::PoseEstimationInitialization) &&
			(neighbor_state[neighbor] != OptimizerState::RotationEstimation);
		all_finished_pose_estimation &= other_robot_finished;
	} 
	if(all_finished_pose_estimation){
		return true;
	}

	// neighbors have finished pose estimation
	bool stop_pose_estimation = pose_estimate_finished;
	for(int i = 0; i < optimization_order.size(); i++){
		int other_robot = optimization_order[i];
		if(other_robot != robot_id){
			stop_pose_estimation &= neighbors_pose_estimate_finished[other_robot] ||
				neighbor_state[other_robot] > optimizer_state;
		}
	}
	return stop_pose_estimation;
}


void distributedMapping::incrementalInitialGuessUpdate(){
	// update poses values
	initial_values->update(optimizer->currentEstimate());
	// incremental update
	for(size_t k = 0; k < local_pose_graph->size(); k++){
		KeyVector keys = local_pose_graph->at(k)->keys();
		if(keys.size() == 2){
			auto between_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(local_pose_graph->at(k));
			Symbol key0 = between_factor->keys().at(0);
			Symbol key1 = between_factor->keys().at(1);
			int index0 = key0.index();
			int robot0 = key0.chr();
			int index1 = key1.index();
			int robot1 = key1.chr();
			if(!optimizer->currentEstimate().exists(key1) && (robot0 == robot1) && (index1 = index0 + 1)){
				// get previous pose
				auto previous_pose = initial_values->at<Pose3>(key0);
				// compose previous pose and measurement
				auto current_pose = previous_pose * between_factor->measured();
				// update pose in initial guess
				initial_values->update(key1, current_pose);
			}
		}
	}

	// aggregate estimates
	global_path.poses.clear();
	pcl::PointCloud<PointPose3D>::Ptr poses_3d_cloud(new pcl::PointCloud<PointPose3D>());
	for(const Values::ConstKeyValuePair &key_value: *initial_values){
		// update key poses
		Symbol key = key_value.key;
		int index = key.index();
		Pose3 pose = initial_values->at<Pose3>(key);

		PointPose3D pose_3d;
		pose_3d.x = pose.translation().x();
		pose_3d.y = pose.translation().y();
		pose_3d.z = pose.translation().z();
		pose_3d.intensity = key.index();
		poses_3d_cloud->push_back(pose_3d);

		updateGlobalPath(pose);
	}

	if(pub_keypose_cloud->get_subscription_count() != 0){
		// publish global map
		sensor_msgs::msg::PointCloud2 keypose_cloud_msg;
		pcl::toROSMsg(*poses_3d_cloud, keypose_cloud_msg);
		keypose_cloud_msg.header.stamp = this->now();
		keypose_cloud_msg.header.frame_id = world_frame_;
		pub_keypose_cloud->publish(keypose_cloud_msg);
	}

	if(pub_global_path->get_subscription_count() != 0){
		global_path.header.stamp = this->now();
		global_path.header.frame_id = world_frame_;
		pub_global_path->publish(global_path);
	}
}

void distributedMapping::endOptimization(){
	// retract to global frame
	if(prior_owner == robot_id){
		optimizer->retractPose3GlobalWithOffset(anchor_offset);
	} else {
		optimizer->retractPose3GlobalWithOffset(neighbors_anchor_offset[prior_owner]);
	}

	// update estimates
	incrementalInitialGuessUpdate();

	lowest_id_included = lowest_id_to_included;
}

void distributedMapping::changeOptimizerState(const OptimizerState& state) {
    // Switch statement for handling different optimizer states with logging
    switch (state) {
        case OptimizerState::Idle:
            RCLCPP_INFO(this->get_logger(), "<%d> Idle", robot_id);
            break;
        case OptimizerState::Start:
            RCLCPP_INFO(this->get_logger(), "<%d> Start", robot_id);
            break;
        case OptimizerState::Initialization:
            RCLCPP_INFO(this->get_logger(), "<%d> Initialization", robot_id);
            break;
        case OptimizerState::RotationEstimation:
            RCLCPP_INFO(this->get_logger(), "<%d> RotationEstimation", robot_id);
            break;
        case OptimizerState::PoseEstimationInitialization:
            RCLCPP_INFO(this->get_logger(), "<%d> PoseEstimationInitialization", robot_id);
            break;
        case OptimizerState::PoseEstimation:
            RCLCPP_INFO(this->get_logger(), "<%d> PoseEstimation", robot_id);
            break;
        case OptimizerState::End:
            RCLCPP_INFO(this->get_logger(), "<%d> End", robot_id);
            break;
        case OptimizerState::PostEndingCommunicationDelay:
            RCLCPP_INFO(this->get_logger(), "<%d> PostEndingCommunicationDelay", robot_id);
            break;
    }

    // Update the optimizer state
    optimizer_state = state;

    // Update the state message and publish it
    state_msg.data = static_cast<int>(optimizer_state);
    robots[robot_id].pub_optimization_state->publish(state_msg);
}

void distributedMapping::run()
{
	// update optimizer state
	switch(optimizer_state){
		case OptimizerState::Idle:
			if(startOptimizationCondition()){
				current_rotation_estimate_iteration = 0;
				current_pose_estimate_iteration = 0;
				// neighbors_within_communication_range.clear();
				neighbors_rotation_estimate_finished.clear();
				neighbors_pose_estimate_finished.clear();
				neighbors_anchor_offset.clear();
				neighbors_estimation_done.clear();
				latest_change = -1;
				steps_without_change = 0;
				lowest_id_to_included = lowest_id_included;
				neighbors_started_optimization.clear();
				if(prior_added){
					optimizer->removePrior();
					prior_added = false;
				}
				optimization_steps = 0;
				changeOptimizerState(OptimizerState::Start);
			} else {
				changeOptimizerState(OptimizerState::Idle);
			}
			state_msg.data = (int)optimizer_state;
			robots[robot_id].pub_optimization_state->publish(state_msg);
			break;

		case OptimizerState::Start:
			initializePoseGraphOptimization();
			optimization_steps++;
			if(in_order){
				changeOptimizerState(OptimizerState::Initialization);
			} else {
				changeOptimizerState(OptimizerState::Idle);
			}
			break;

		case OptimizerState::Initialization:
			changeOptimizerState(OptimizerState::RotationEstimation);
			rotation_estimate_start = false;
			optimization_steps++;
			break;

		case OptimizerState::RotationEstimation:
			if(rotationEstimationStoppingBarrier()){
				changeOptimizerState(OptimizerState::PoseEstimationInitialization);
			}
			if(current_rotation_estimate_iteration > fail_safe_steps_){
				removeInactiveNeighbors();
			}
			failSafeCheck();
			optimization_steps++;
			break;

		case OptimizerState::PoseEstimationInitialization:
			initializePoseEstimation();
			pose_estimate_start = false;
			optimization_steps++;
			changeOptimizerState(OptimizerState::PoseEstimation);
			break;

		case OptimizerState::PoseEstimation:
			if(poseEstimationStoppingBarrier()){
				changeOptimizerState(OptimizerState::End);
			}
			failSafeCheck();
			optimization_steps++;
			break;

		case OptimizerState::End:
			endOptimization();
			changeOptimizerState(OptimizerState::PostEndingCommunicationDelay);
			optimization_steps++;
			break;

		case OptimizerState::PostEndingCommunicationDelay:
			changeOptimizerState(OptimizerState::Idle);
			optimization_steps++;
			break;
	}
}