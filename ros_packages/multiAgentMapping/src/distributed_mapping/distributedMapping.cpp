#include "multiAgentMapping/distributed_mapping/distributedMapping.hpp"

void distributedMapping::globalDescriptorHandler(
	const multi_agent_mapping::msg::GlobalDescriptor::SharedPtr msg,
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