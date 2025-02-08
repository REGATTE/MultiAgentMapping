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
		id_, id, msg->estimation_done, 
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
				RCLCPP_ERROR("Stopping pose estimation %d: %s.", robot_id, ex.what());
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
				id_, 
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
						robots[other_robot].estimate_msg.anchor_offset.push_back(anchor_offset.vector()[i]);
					}
					robots[other_robot].estimate_msg.initialized = optimizer->isRobotInitialized();
					robots[other_robot].estimate_msg.receiver_id = other_robot;
					robots[other_robot].estimate_msg.estimation_done = estimation_done;
					robots[id_].pub_neighbor_pose_estimates.publish(robots[other_robot].estimate_msg);
				}
			}
			// send pose optimization state
			pose_estimate_start = false;
			optimizer->clearNeighboringRobotInit();
		}
		// send optimization state
		state_msg.data = estimation_done? 1:0;
		robots[robot_id].pub_pose_Estiamte_state->publish(state_msg);
	}
}

