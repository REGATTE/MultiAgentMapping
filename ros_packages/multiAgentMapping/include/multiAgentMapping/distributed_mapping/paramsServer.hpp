#ifndef _PARAM_SERVER_H_
#define _PARAM_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <std_msgs/msg/int8.hpp>
// msg
#include "multi_agent_mapping/msg/neighbor_estimate.hpp"
#include "multi_agent_mapping/msg/loop_info.hpp"
#include "multi_agent_mapping/msg/global_descriptor.hpp"
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

enum class LiDARType {
    VELODYNE
};

enum class DescriptorType {
    LidarIris
};
enum class OptimizerState { 
    Idle, 
    Start, 
    Initialization, 
    RotationEstimation, 
	PoseEstimationInitialization, 
    PoseEstimation, 
    End, 
    PostEndingCommunicationDelay 
};

struct singleRobot {
	/*** robot information ***/
	int robot_id; // robot id
	std::string robot_name; // robot name, for example, 'scout_1_1, scout_2_2,...'
	std::string odom_frame_; // odom frame

    /*** ROS2 subscribers and publishers ***/
    // Mapping
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_optimization_state;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_pose_estimate_state;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_rotation_estimate_state;
    rclcpp::Subscription<multi_agent_mapping::msg::NeighborEstimate>::SharedPtr sub_neighbor_rotation_estimates;
    rclcpp::Subscription<multi_agent_mapping::msg::NeighborEstimate>::SharedPtr sub_neighbor_pose_estimates;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_optimization_state;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_pose_estimate_state;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_rotation_estimate_state;
    rclcpp::Publisher<multi_agent_mapping::msg::NeighborEstimate>::SharedPtr pub_neighbor_rotation_estimates;
    rclcpp::Publisher<multi_agent_mapping::msg::NeighborEstimate>::SharedPtr pub_neighbor_pose_estimates;

    // Loop closure
    rclcpp::Subscription<multi_agent_mapping::msg::LoopInfo>::SharedPtr sub_loop_info;
    rclcpp::Publisher<multi_agent_mapping::msg::LoopInfo>::SharedPtr pub_loop_info;

    // Descriptors
    rclcpp::Subscription<multi_agent_mapping::msg::GlobalDescriptor>::SharedPtr sub_descriptors;
    rclcpp::Publisher<multi_agent_mapping::msg::GlobalDescriptor>::SharedPtr pub_descriptors;

	/*** other ***/
    rclcpp::Time time_cloud_input_stamp; // recent keyframe timestamp
	double time_cloud_input; // and its double type
	multi_agent_mapping::msg::NeighborEstimate estimate_msg; // pose and rotation estimate msg
	pcl::PointCloud<PointPose3D>::Ptr keyframe_cloud; // recent keyframe pointcloud
	std::vector<pcl::PointCloud<PointPose3D>> keyframe_cloud_array; // and its array
	Pose3 prior_odom; // piror factor
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
        gtsam::Pose3 pclPointTogtsamPose3(
            PointPose6D point
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

        // keypose threshold
        float surroundingkeyframeAddingDistThreshold;
        float surroundingkeyframeAddingAngleThreshold;

        // CPU params
        int onboard_cpu_cores_num_; // cores number of onboard unit
		float loop_closure_process_interval_; // interval of detecting loop (in second)
		float map_publish_interval_; // interval of publish global maps (in second)
		float mapping_process_interval_; // interval of optimization (in second)
    
        // Mapping
		bool global_optimization_enable_; // enable distributed DGS
		bool use_pcm_; // enable pairwise consistency maximization (PCM)
		float pcm_threshold_; // confidence probability for PCM (i.e., 0.01, 0.05, 0.1, 0.25, 0.5, 0.75)
		int optimization_maximum_iteration_; // maximum iterations time of optimization
		bool use_between_noise_; // use between noise flag
		int fail_safe_steps_; // steps of fail safe to abort (depend on both fail_safe_wait_time_ and mapping_process_interval_)
		float fail_safe_wait_time_; // wait time for fail safe (in second)
		float rotation_estimate_change_threshold_;  // difference between rotation estimate provides an early stopping condition
		float pose_estimate_change_threshold_; // difference between pose estimate provides an early stopping condition
		float gamma_; // gamma value for over relaxation methods
		bool use_flagged_init_; // to use flagged initialization or not
		bool use_landmarks_; // use landmarks -- landmarks are given symbols as upper case of robot name
		bool use_heuristics_; // use heuristics-based algorithm for the max-clique solver

        // downsample
		float map_leaf_size_; // scan to map matching downsample rate (default 0.4)
		float descript_leaf_size_; // descriptor downsample rate (default 0.1)

        // loop closure
		bool intra_robot_loop_closure_enable_; // enable to search intra-robot loop closre with global descriptor
		bool inter_robot_loop_closure_enable_; // enable to search intra-robot loop closre with global descriptor
		DescriptorType descriptor_type_num_; // descriptor type: ScanContext, LidarIris, M2DP
		int knn_candidates_; // k nearest neighbor search of row key
		int exclude_recent_frame_num_; // exclude recent keyframe in intra-robot loop closure
		float search_radius_; // radius of radius search based intra-robot loop closure
		int match_mode_; // iris-feature matching mode, (i.e., 0, 1, 2; default 2) 
		int iris_row_; // iris-image row
		int iris_column_; // iris-image column
		float descriptor_distance_threshold_; // iris-feature match threshold
		int ransac_maximum_iteration_; // RANSAC maximum iteration time
		float ransac_threshold_; // RANSAC threshold (rate: [0 1])
		float ransac_outlier_reject_threshold_; // RANSAC outlier rejection distancce
		int history_keyframe_search_num_; // number of history frames in submap for scan-to-map matching
		float fitness_score_threshold_; // ICP fitness score threshold

        // visualization
		float global_map_visualization_radius_; // radius of radius search based intra-robot loop closure

		// Save pcd
		std::string save_directory_;
};

#endif // _PARAM_SERVER_H_