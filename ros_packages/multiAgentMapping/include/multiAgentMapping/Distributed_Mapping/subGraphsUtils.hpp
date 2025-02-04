#ifndef _SUBGRAPHS_UTILS_H_
#define _SUBGRAPHS_UTILS_H_

#include "multiAgentMapping/LIO_SAM/utility.hpp"
#include "distributed_mapper/distributed_mapper.hpp"
#include "distributed_mapper/distributed_mapper_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// gtsam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <tf2_eigen/tf2_eigen.hpp>

using namespace std;
using namespace gtsam;

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
    pcl::PointCloud<PointPose3D>::Ptr robot_keyframe_cloud; //recent keyframe pointcloud
    std::vector<pcl::PointCloud<PointPose3D>> robot_keyframe_cloud_array; // Array of keyframe clouds
    Pose3 prior_odom; // Prior factor

};

enum class LiDARType { VELODYNE, LIVOX };
enum class DescriptorType { ScanContext, LidarIris, M2DP };
enum class OptimizerState { Idle, Start, Initialization, RotationEstimation, 
	PoseEstimationInitialization, PoseEstimation, End, PostEndingCommunicationDelay };

class subGraphMapping : public rclcpp::Node {
    public:

        subGraphMapping();

        void processKeyframeForMapping(
            const gtsam::Pose3& pose_to,
            const pcl::PointCloud<PointPose3D>::Ptr& frame_to,
            const rclcpp::Time& timestamp
        );

        void updateLocalPath(
            const PointPose6D& pose
        );

        void updateGlobalPath(
            const Pose3& pose
        );

        gtsam::Pose3 pclPointTogtsamPose3(PointPose6D point);

    protected:
        std::map<int, singleRobot> robot_info;

        int number_of_robots_;

        std::string robot_namespace;
        int robot_id;

        std::string world_frame_;
        std::string odom_frame_;

        nav_msgs::msg::Path local_path; // path in local frame
		nav_msgs::msg::Path global_path; // path in global frame

        pcl::PointCloud<PointPose3D>::Ptr keyposes_cloud_3d; // 3-dof keyposes in local frame
		pcl::PointCloud<PointPose6D>::Ptr keyposes_cloud_6d; // 6-dof keyposes in local frame

        // local pose graph optimization
        gtsam::ISAM2* isam2;
        NonlinearFactorGraph isam2_graph;               // local pose graph for isam2
        gtsam::Values isam2_initial_values;             // local initial values for isam2
        gtsam::Values isam2_current_estimate;          // current estimates for isam2
        Pose3 isam2_keypose_estimate;                   // Keypose estimate for isam2
        std::shared_ptr<gtsam::Values> initial_values;

        std::shared_ptr<NonlinearFactorGraph> local_pose_graph; // local pose graph for distributed mapping

        // noise model
        noiseModel::Isotropic::shared_ptr prior_noise;
        noiseModel::Diagonal::shared_ptr odometry_noise;

        std::shared_ptr<NonlinearFactorGraph> local_pose_graph_no_filtering; // pose graph without pcm

        // distributed pairwise consistency maximization
		robot_measurements::RobotLocalMap robot_local_map; // local loop closures and transform 
		robot_measurements::RobotLocalMap robot_local_map_backup; // backups in case of abort
};

#endif // _SUBGRAPHS_UTILS_H_