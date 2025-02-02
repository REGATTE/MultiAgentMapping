#ifndef _SUBGRAPHS_UTILS_H_
#define _SUBGRAPHS_UTILS_H_

#include "multiAgentMapping/LIO_SAM/utility.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// gtsam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

#include <tf2_eigen/tf2_eigen.hpp>

using namespace std;
using namespace gtsam;

typedef pcl::PointXYZI PointPose3D;

class subGraphMapping : public rclcpp::Node {
    public:

        subGraphMapping();

        pcl::PointCloud<PointPose3D>::Ptr keyframe_cloud; // recent keyframe pointcloud
	    std::vector<pcl::PointCloud<PointPose3D>> keyframe_cloud_array; // and its array

        void performDistributedMapping(
            const gtsam::Pose3& pose_to,
            const pcl::PointCloud<PointPose3D>::Ptr& frame_to,
            const rclcpp::Time& timestamp
        );
};

#endif // _SUBGRAPHS_UTILS_H_