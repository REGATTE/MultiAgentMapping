#ifndef _DISTRIBUTED_MAPPING_H_
#define _DISTRIBUTED_MAPPING_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2/LinearMath/Quaternion.h>  // For Quaternion operations
#include <tf2_ros/transform_listener.h>  // For listening to transforms
#include <tf2_ros/transform_broadcaster.h>  // For broadcasting transforms
#include <geometry_msgs/msg/transform_stamped.hpp>  // For publishing transformations
#include <geometry_msgs/msg/pose_with_covariance.hpp>

// include headers
#include "multiAgentMapping/distributed_mapping/paramsServer.hpp"
#include "multiAgentMapping/distributed_mapping/lidarIrisDescriptor.hpp"
// messages
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

#include <gtsam/nonlinear/ISAM2.h>
#include <flann/flann.hpp>
#include <thread>
#include <deque>
#include <map>

#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

class distributedMapping : public paramsServer {
    public:
        distributedMapping();
        ~distributedMapping();

        pcl::PointCloud<PointPose3D>::Ptr getLocalKeyposesCloud3D();
        pcl::PointCloud<PointPose6D>::Ptr getLocalKeyposesCloud6D();
        pcl::PointCloud<PointPose3D> getLocalKeyframe(const int& index);

        Pose3 getLatestEstimate();
        void processkeyframe(
            const Pose3& pose_to,
            const pcl::PointCloud<PointPose3D>::Ptr frame_to,
            const rclcpp::Time& timestamp
        );
        bool saveFrame(
            const Pose3& pose_to
        );
        void updateLocalPath(
            const PointPose6D& pose
        );
        void updatePoses();
        void makeDescriptors();
        void publishPath();
        void publishTransformation(
            const rclcpp::Time& timestamp
        );
        void loopClosureThread();
        void globalMapThread();
    private:
        void poseCovariance2msg(
            const graph_utils::PoseWithCovariance& pose,
            geometry_msgs::msg::PoseWithCovariance& msg
        );
        void msg2poseCovariance(
            const geometry_msgs::msg::PoseWithCovariance& msg,
            graph_utils::PoseWithCovariance& pose
        );
        void globalDescriptorHandler(
            const multi_agent_mapping::msg::GlobalDescriptor& msg,
            int& robot_id
        );
        void loopInfoHnadler(
            const multi_agent_mapping::msg::LoopInfo& msg,
            int& robot_id
        );
        void optStateHandler(
            const std_msgs::msg::Int8& msg,
            int& robot_id
        );
        void rotationStateHandler(
            const std_msgs::msg::Int8& msg,
            int& robot_id
        );
        void poseStateHandler(
            const std_msgs::msg::Int8& msg,
            int& robot_id
        );
        void neighbotRotationHandler(
            const multi_agent_mapping::msg::NeighborEstimate& msg,
            int& robot_id
        );
        void neighborPoseHandler(
			const multi_agent_mapping::msg::NeighborEstimate& msg,
			int& id
		);
		void updatePoseEstimateFromNeighbor(
			const int& rid,
			const Key& key,
			const graph_utils::PoseWithCovariance& pose
        );
        bool startOptimizationCondition();
        void updateOptimizer();
        void outliersFiltering();
        void computeOptimizationOrder();
        void initalizePoseGraphOptimization();
        bool rotationEstimationStoppingBarrier();
        void abortOptimization(
            const bool& log_info
        );
        void removeInactiveNeighbors();
        void faileSafeCheck();
        void initializePoseEstimation();
		bool poseEstimationStoppingBarrier();
		void updateGlobalPath(
			const Pose3& pose_in
        );
		void incrementalInitialGuessUpdate();
		void endOptimization();
		void changeOptimizerState(
			const OptimizerState& state
        );
		void run(
			const rclcpp::TimerBase::SharedPtr& timer_event
		);
		void performRSIntraLoopClosure();
		int detectLoopClosureDistance(
			const int& cur_ptr
        );
		void performIntraLoopClosure();
		void calculateTransformation(
			const int& loop_key_cur,
			const int& loop_key_pre
        );
		void loopFindNearKeyframes(
			pcl::PointCloud<PointPose3D>::Ptr& near_keyframes,
			const int& key, const int& search_num
        );
		void performInterLoopClosure();
		void performExternLoopClosure();
		void loopFindGlobalNearKeyframes(
			pcl::PointCloud<PointPose3D>::Ptr& near_keyframes,
			const int& key, const int& search_num
        );
		void publishGlobalMap();
		void publishLoopClosureConstraint();
    
    public:
        mutex lock_on_call; // lock on odometry
    
    private:
        vector<singleRobot> robots;

        /*** ROS2 subscribers and publishers ***/
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_loop_closure_constraints;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_of_scan2map;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_of_scan2map;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_map;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_path;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_keypose_cloud;

        // message information
        pcl::PointCloud<PointPose3D>::Ptr cloud_for_decript_ds; // input cloud for descriptor
		deque<pair<int, multi_agent_mapping::msg::GlobalDescriptor>> store_descriptors;

        std_msgs::msg::Int8 state_msg; // optimization state handler

        multi_agent_mapping::msg::GlobalDescriptor global_descriptor_msg; // descriptor message

        // path define in local and global frame
        nav_msgs::msg::Path local_path;
        nav_msgs::msg::Path global_path;

        /*** downsample filter ***/
		pcl::VoxelGrid<PointPose3D> downsample_filter_for_descriptor;
		pcl::VoxelGrid<PointPose3D> downsample_filter_for_intra_loop;
		pcl::VoxelGrid<PointPose3D> downsample_filter_for_inter_loop;
		pcl::VoxelGrid<PointPose3D> downsample_filter_for_inter_loop2;
		pcl::VoxelGrid<PointPose3D> downsample_filter_for_inter_loop3;

        /*** distributed loopclosure ***/
		int intra_robot_loop_ptr; // current position pointer for intra-robot loop
		int inter_robot_loop_ptr; // current position pointer for inter-robot loop

		bool intra_robot_loop_close_flag; // intra-robot loop is detected

		unique_ptr<scan_descriptor> keyframe_descriptor; // descriptor for keyframe pointcloud

		deque<multi_agent_mapping::msg::LoopInfo> loop_closures_candidates; // loop closures need to verify

		// radius search for intra-robot loop closure
		pcl::PointCloud<PointPose3D>::Ptr copy_keyposes_cloud_3d; // copy of local 3-dof keyposes
		pcl::PointCloud<PointPose6D>::Ptr copy_keyposes_cloud_6d; // copy of local 6-dof keyposes

		pcl::KdTreeFLANN<PointPose3D>::Ptr kdtree_history_keyposes; // kdtree for searching history keyposes

		map<int, int> loop_indexs;
		map<Symbol, Symbol> loop_indexes;

		/*** noise model ***/
		noiseModel::Diagonal::shared_ptr odometry_noise; // odometry factor noise
		noiseModel::Diagonal::shared_ptr prior_noise; // prior factor noise

		/*** local pose graph optmazition ***/
		ISAM2 *isam2; // isam2 optimizer

		NonlinearFactorGraph isam2_graph; // local pose graph for isam2
		Values isam2_initial_values; // local initial values for isam2

		Values isam2_current_estimates; // current estimates for isam2
		Pose3 isam2_keypose_estimate; // keypose estimate for isam2

		pcl::PointCloud<PointPose3D>::Ptr keyposes_cloud_3d; // 3-dof keyposes in local frame
		pcl::PointCloud<PointPose6D>::Ptr keyposes_cloud_6d; // 6-dof keyposes in local frame

		/*** distributed pose graph optmazition ***/
		rclcpp::TimerBase::SharedPtr distributed_mapping_thread; // thread for running distributed mapping
		boost::shared_ptr<distributed_mapper::DistributedMapper> optimizer; // distributed mapper (DGS)

		int steps_of_unchange_graph; // stop optimization 

		// measurements
		boost::shared_ptr<NonlinearFactorGraph> local_pose_graph; // pose graph for distributed mapping
		boost::shared_ptr<Values> initial_values; // initial values for distributed mapping
		GraphAndValues graph_values_vec; // vector of pose graph and initial values

		bool graph_disconnected; // pose graph is not connected to others

		int lowest_id_included; // lowest id in this robot
		int lowest_id_to_included; // lowest id to be included in this robot
		int prior_owner; // the robot that own prior factor
		bool prior_added; // this robot have add prior factor

		gtsam::Matrix adjacency_matrix; // adjacency matrix of robot team
		vector<int> optimization_order; // optimization order of robot team
		bool in_order; // this robot in optimization order

		// this robot
		OptimizerState optimizer_state; // current state of optimizer
		int optimization_steps; // steps in optimization
		bool sent_start_optimization_flag; // ready for optimization

		int current_rotation_estimate_iteration; // current iteration time of rotation estimate
		int current_pose_estimate_iteration; // current iteration time of pose estimate

		double latest_change; // latest change of estimate
		int steps_without_change; // setps of estimate without change

		bool rotation_estimate_start; // rotation estimate is start
		bool pose_estimate_start; // pose estimate is start
		bool rotation_estimate_finished; // rotation estimate is finished
		bool pose_estimate_finished; // pose estimate is finished
		bool estimation_done; // estimate is done

		Point3 anchor_offset, anchor_point; // anchor offset

		// neighbors
		set<char> neighboring_robots; // neighbors (name) within communication range
		set<int> neighbors_within_communication_range; // neighbors (id) within communication range
		map<int, bool> neighbors_started_optimization; // neighbors ready for optimization
		map<int, OptimizerState> neighbor_state; // current state of neighbors optimizer

		map<int, bool> neighbors_rotation_estimate_finished; // neighbors rotation estimate is finished
		map<int, bool> neighbors_pose_estimate_finished; // neighbors pose estimate is finished
		map<int, bool> neighbors_estimation_done; // neighbors estimate is done

		map<int, int> neighbors_lowest_id_included; // lowest id in neighbors
		map<int, Point3> neighbors_anchor_offset; // neighbors anchor offset

		// distributed pairwise consistency maximization
		robot_measurements::RobotLocalMap robot_local_map; // local loop closures and transform 
		robot_measurements::RobotLocalMap robot_local_map_backup; // backups in case of abort

		boost::shared_ptr<NonlinearFactorGraph> local_pose_graph_no_filtering; // pose graph without pcm

		map<int, graph_utils::Trajectory> pose_estimates_from_neighbors; // pose estimates of neighbors
		set<Key> other_robot_keys_for_optimization; // keys of neighbors for optimization

		set<pair<Key, Key>> accepted_keys, rejected_keys; // accepted and rejected pairs
		int measurements_accepted_num, measurements_rejected_num;
};

#endif // _DISTRIBUTED_MAPPING_H_