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
struct PointPose6D
{
    float x;
	float y;
	float z;
	float intensity;
    float roll;
    float pitch;
    float yaw;
    double time;
};
POINT_CLOUD_REGISTER_POINT_STRUCT  (PointPose6D,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

struct singleRobot {
    int robot_id;

    std::string robot_namespace;
    std::string odom_frame_;

    rclcpp::Time point_cloud_input_stamp;
    double point_cloud_input_time;
    pcl::PointCloud<PointPose3D>::Ptr keyframe_cloud; //recent keyframe pointcloud
    std::vector<pcl::PointCloud<PointPose3D>> keyframe_cloud_array; // Array of keyframe clouds
    Pose3 prior_odom; // Prior factor

};

enum class LiDARType { VELODYNE, LIVOX };
enum class DescriptorType { ScanContext, LidarIris, M2DP };
enum class OptimizerState { Idle, Start, Initialization, RotationEstimation, 
	PoseEstimationInitialization, PoseEstimation, End, PostEndingCommunicationDelay };

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

    protected:
        int number_of_robots_;

        std::string robot_namespace;
        int robot_id;

        std::string world_frame_;
        std::string odom_frame_;
};

#endif // _SUBGRAPHS_UTILS_H_