#include "multiAgentMapping/distributedMapping/distributedMapping.hpp"

distributedMapping::distributedMapping() : subGraphParams() {}

bool distributedMapping::saveFrame(const Pose3& pose_to){
    if(keyposes_cloud_3d -> empty()){
        return true;
    }

    auto last_keypose = pclPointTogtsamPose3(keyposes_cloud_6d -> back());
    auto pose_increment = last_keypose.inverse() * pose_to;

    float x = pose_increment.translation().x();
	float y = pose_increment.translation().y();
	float z = pose_increment.translation().z();
	float roll = pose_increment.rotation().roll();
	float pitch = pose_increment.rotation().pitch();
	float yaw = pose_increment.rotation().yaw();

	// select keyframe
	if(abs(roll) < keyframe_angle_threshold_ && abs(pitch) < keyframe_angle_threshold_ && 
		abs(yaw) < keyframe_angle_threshold_ && sqrt(x*x + y*y + z*z) < keyframe_distance_threshold_)
	{
		return false;
	}

	return true;
}