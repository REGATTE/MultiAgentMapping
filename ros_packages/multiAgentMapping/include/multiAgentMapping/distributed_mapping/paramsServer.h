#ifndef _PARAM_SERVER_H_
#define _PARAM_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform.hpp>
// msg
#include "multi_agent_mapping/msg/neighbor_estimate.hpp"
// pcl
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// mapping
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
// LIO_SAM utility.hpp
#include "multiAgentMapping/LIO_SAM/utility.hpp"

using namespace gtsam;
using namespace std;

typedef pcl::PointXYZI PointPose3D;
struct PointPose6D{
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
	/*** robot information ***/
	int robot_id; // robot id
	std::string robot_name; // robot name, for example, 'scout_1_1, scout_2_2,...'
	std::string odom_frame_; // odom frame

	/*** other ***/
    rclcpp::Time time_cloud_input_stamp; // recent keyframe timestamp
	double time_cloud_input; // and its double type
	multi_agent_mapping::msg::NeighborEstimate estimate_msg; // pose and rotation estimate msg
	pcl::PointCloud<PointPose3D>::Ptr keyframe_cloud; // recent keyframe pointcloud
	std::vector<pcl::PointCloud<PointPose3D>> keyframe_cloud_array; // and its array
	Pose3 piror_odom; // piror factor
};

class paramsServer : public rclcpp::Node {
    public:
        paramsServer();
        Eigen::Affine3f gtsamPoseToAffine3f(
            gtsam::Pose3 pose
        );
        geometry_msgs::msg::Transform gtsamPoseToTransform(
            gtsam::Pose3 pose
        );
        gtsam::Pose3 transformToGtsamPose(
            const geometry_msgs::msg::Transform& pose
        );
        pcl::PointCloud<PointPose3D>::Ptr transformPointCloud(
            pcl::PointCloud<PointPose3D> cloud_input,
            PointPose6D* pose
        );
        pcl::PointCloud<PointPose3D>::Ptr transformPointCloud(
            pcl::PointCloud<PointPose3D> cloud_input,
            gtsam::Pose3 pose
        );
    
    protected:
        int number_of_robots_; // number of robots in the sim

        // robot info
        std::string robot_name;
        int robot_id;

        // robot frame name
        std::string world_frame_;
        std::string odom_frame_;

        LiDARType sensor_;
        int n_scan;

        // loop closure
		bool intra_robot_loop_closure_enable_; // enable to search intra-robot loop closre with global descriptor
		bool inter_robot_loop_closure_enable_; // enable to search inter-robot loop closre with global descriptor

        
}

#endif // _PARAM_SERVER_H_