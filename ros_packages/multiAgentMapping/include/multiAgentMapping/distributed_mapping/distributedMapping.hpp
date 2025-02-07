#ifndef _DISTRIBUTED_MAPPING_H_
#define _DISTRIBUTED_MAPPING_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int8.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>  // For Quaternion operations
#include <tf2_ros/transform_listener.h>  // For listening to transforms
#include <tf2_ros/transform_broadcaster.h>  // For broadcasting transforms
#include <flann/flann.hpp>
#include <thread>
#include <deque>
// include headers
#include "multiAgentMapping/distributed_mapping/paramsServer.hpp"
#include "multiAgentMapping/distributed_mapping/lidarIrisDescriptor.hpp"
// include messages
#include "multi_agent_mapping/msg/neighbor_estimate.hpp"
#include "multi_agent_mapping/msg/global_descriptor.hpp"
#include "multi_agent_mapping/msg/loop_info.hpp"
// pcl
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
// distributed mapper colcon
#include "distributed_mapper/distributed_mapper.hpp"
#include "distributed_mapper/distributed_mapper_utils.hpp"
// gtsam
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <fstream>
#include <iostream>

using namespace gtsam;
using namespace std;

class distributedMapping : public paramsServer{
    public:
        distributedMapping();
        ~distributedMapping();
        pcl::PointCloud<PointPose3D>::Ptr getLocalKeyposesCloud3D();
		pcl::PointCloud<PointPose6D>::Ptr getLocalKeyposesCloud6D();
        pcl::PointCloud<PointPose3D> getLocalKeyframe(const int& index);
        Pose3 getLatestEstimate();
        void processKeyframesAndPose(
            const Pose3& pose_to,
            const pcl::PointCloud<PointPose3D>::Ptr frame_to,
            const rclcpp::Time& timestamp
        );
        bool saveFrame(
            const Pose3& pose_to
        );
        void updateLocalPath(
			const PointPose6D& pose);
		bool updatePoses();
		void makeDescriptors();
		void publishPath();
		void publishTransformation(
			const rclcpp::Time& timestamp);
		void loopClosureThread();
		void globalMapThread();
};

#endif // _DISTRIBUTED_MAPPING_H_