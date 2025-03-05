#include "multiAgentMapping/distributed_mapping/distributedMapping.hpp"

distributedMapping::distributedMapping(const rclcpp::NodeOptions & options) 
    : paramsServer("distributed_mapping", options) {
    std::string log_name = robot_name + "/distributed_mapping";
    // Get logger instance
    auto logger = rclcpp::get_logger(log_name);
    // Get log directory
    std::string log_dir = std::string(std::getenv("HOME")) + "/mutliAgentMapping_log";
    // Log initialization message
    RCLCPP_INFO(logger, "Distributed mapping class initialization. Log name: %s, Log directory: %s",  log_name.c_str(), log_dir.c_str());

    singleRobot robot;
    for (int it=0; it<number_of_robots_; it++){
        // robot info
        robot.robot_id = it;
        robot.robot_name = "/a";
        robot.robot_name[1] += it ;
        robot.odom_frame_ = robot.robot_name + "/" + odom_frame_;

        // this robot
        if (it == robot_id){
            RCLCPP_INFO(this->get_logger(), "Robot ID: %d", robot_id);
            if(intra_robot_loop_closure_enable_ || inter_robot_loop_closure_enable_){
                // publish global descriptors
                robot.pub_descriptors = this->create_publisher<multi_agent_mapping::msg::GlobalDescriptor>(
                    robot.robot_name + "/distributedMapping/globalDescriptors", 
                    rclcpp::QoS(50).reliable()
                );
                RCLCPP_INFO(this->get_logger(), "Publishing to: %s", (robot.robot_name + "/distributedMapping/globalDescriptors").c_str());
                // publish Loop Info
                robot.pub_loop_info = this->create_publisher<multi_agent_mapping::msg::LoopInfo>(
                    robot.robot_name + "/distributedMapping/loopInfo", 
                    rclcpp::QoS(50).reliable()
                );
                RCLCPP_INFO(this->get_logger(), "Publishing to: %s", (robot.robot_name + "/distributedMapping/loopInfo").c_str());
            }
            if(global_optimization_enable_){
                // publish optimization state
                robot.pub_optimization_state = this->create_publisher<std_msgs::msg::Int8>(
                    robot.robot_name + "/distributedMapping/optimizationState", 
                    rclcpp::QoS(50).reliable()
                );
                robot.pub_rotation_estimate_state = this->create_publisher<std_msgs::msg::Int8>(
                    robot.robot_name + "/distributedMapping/rotationEstimateState", 
                    rclcpp::QoS(50).reliable()
                );
                robot.pub_pose_estimate_state = this->create_publisher<std_msgs::msg::Int8>(
                    robot.robot_name + "/distributedMapping/poseEstimateState", 
                    rclcpp::QoS(50).reliable()
                );
                robot.pub_neighbor_rotation_estimates = this->create_publisher<multi_agent_mapping::msg::NeighborEstimate>(
                    robot.robot_name + "/distributedMapping/neighborRotationEstimates", 
                    rclcpp::QoS(50).reliable()
                );
                robot.pub_neighbor_pose_estimates = this->create_publisher<multi_agent_mapping::msg::NeighborEstimate>(
                    robot.robot_name + "/distributedMapping/neighborPoseEstimates", 
                    rclcpp::QoS(50).reliable()
                );
            }
        } else {
            if (intra_robot_loop_closure_enable_ || inter_robot_loop_closure_enable_) {
                // Global Descriptor subscription
                robot.sub_descriptors = this->create_subscription<multi_agent_mapping::msg::GlobalDescriptor>(
                    robot.robot_name + "/distributedMapping/globalDescriptors",
                    rclcpp::QoS(50).reliable(),
                    [this, it](const multi_agent_mapping::msg::GlobalDescriptor::SharedPtr msg) {
                        RCLCPP_INFO(this->get_logger(), 
                            "[Descriptor Handler TRIGGERED] Received global descriptor from robot %d on topic: %s",
                            it, 
                            (robots[it].robot_name + "/distributedMapping/globalDescriptors").c_str());
                        this->globalDescriptorHandler(msg, it);
                    });
                
                RCLCPP_INFO(this->get_logger(), "[SUBSCRIBED] to: %s", (robot.robot_name + "/distributedMapping/globalDescriptors").c_str());

                
                // Loop Info subscription
                robot.sub_loop_info = this->create_subscription<multi_agent_mapping::msg::LoopInfo>(
                    robot.robot_name + "/distributedMapping/loopInfo",
                    rclcpp::QoS(50).reliable(),
                    [this, it](const multi_agent_mapping::msg::LoopInfo::SharedPtr msg) {
                        RCLCPP_INFO(this->get_logger(), 
                            "[Loop Info Handler TRIGGERED] Received loop info from robot %d on topic: %s",
                            it, 
                            (robots[it].robot_name + "/distributedMapping/loopInfo").c_str());
                        this->loopInfoHandler(msg, it);
                    });
                
                RCLCPP_INFO(this->get_logger(), "[SUBSCRIBED] to: %s", (robot.robot_name + "/distributedMapping/loopInfo").c_str());
            }

            if (global_optimization_enable_) {
                robot.sub_optimization_state = this->create_subscription<std_msgs::msg::Int8>(
                    robot.robot_name + "/distributedMapping/optimizationState",
                    rclcpp::QoS(50).reliable(),
                    [this, it](const std_msgs::msg::Int8::SharedPtr msg) {
                        RCLCPP_INFO(this->get_logger(), 
                            "[Optimization State Handler TRIGGERED] Received optimization state from robot %d on topic: %s | State: %d",
                            it, 
                            (robots[it].robot_name + "/distributedMapping/optimizationState").c_str(),
                            msg->data);
                        this->optStateHandler(msg, it);
                    });

                robot.sub_rotation_estimate_state = this->create_subscription<std_msgs::msg::Int8>(
                    robot.robot_name + "/distributedMapping/rotationEstimateState",
                    rclcpp::QoS(50).reliable(),
                    [this, it](const std_msgs::msg::Int8::SharedPtr msg) {
                        RCLCPP_INFO(this->get_logger(), 
                            "[Rotation Estimate State Handler TRIGGERED] Received rotation estimate state from robot %d on topic: %s | State: %d",
                            it, 
                            (robots[it].robot_name + "/distributedMapping/rotationEstimateState").c_str(),
                            msg->data);
                        this->rotationStateHandler(msg, it);
                    });

                robot.sub_pose_estimate_state = this->create_subscription<std_msgs::msg::Int8>(
                    robot.robot_name + "/distributedMapping/poseEstimateState",
                    rclcpp::QoS(50).reliable(),
                    [this, it](const std_msgs::msg::Int8::SharedPtr msg) {
                        RCLCPP_INFO(this->get_logger(), 
                            "[Pose Estimate State Handler TRIGGERED] Received pose estimate state from robot %d on topic: %s | State: %d",
                            it, 
                            (robots[it].robot_name + "/distributedMapping/poseEstimateState").c_str(),
                            msg->data);
                        this->poseStateHandler(msg, it);
                    });

                robot.sub_neighbor_rotation_estimates = this->create_subscription<multi_agent_mapping::msg::NeighborEstimate>(
                    robot.robot_name + "/distributedMapping/neighborRotationEstimates",
                    rclcpp::QoS(50).reliable(),
                    [this, it](const multi_agent_mapping::msg::NeighborEstimate::SharedPtr msg) {
                        RCLCPP_INFO(this->get_logger(), 
                            "[Neighbor Rotation Estimates Handler TRIGGERED] Received neighbor rotation estimates from robot %d on topic: %s",
                            it, 
                            (robots[it].robot_name + "/distributedMapping/neighborRotationEstimates").c_str());
                        this->neighborRotationHandler(msg, it);
                    });

                robot.sub_neighbor_pose_estimates = this->create_subscription<multi_agent_mapping::msg::NeighborEstimate>(
                    robot.robot_name + "/distributedMapping/neighborPoseEstimates",
                    rclcpp::QoS(50).reliable(),
                    [this, it](const multi_agent_mapping::msg::NeighborEstimate::SharedPtr msg) {
                        RCLCPP_INFO(this->get_logger(), 
                            "[Neighbor Pose Estimates Handler TRIGGERED] Received neighbor pose estimates from robot %d on topic: %s",
                            it, 
                            (robots[it].robot_name + "/distributedMapping/neighborPoseEstimates").c_str());
                        this->neighborPoseHandler(msg, it);
                    });
            }   
        }

        robot.time_cloud_input_stamp = rclcpp::Time();
        robot.time_cloud_input = 0.0;

        robot.keyframe_cloud.reset(new pcl::PointCloud<PointPose3D>());
		robot.keyframe_cloud_array.clear();

		robots.push_back(robot);
    }

    // ROS2 Subscriber and publisher global
    // Loop closure visualization
    pub_loop_closure_constraints = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "distributedMapping/loopClosureConstraints", 1
    );
    // Scan2map cloud
    pub_scan_of_scan2map = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "distributedMapping/scanOfScan2map", 1
    );
    pub_map_of_scan2map = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "distributedMapping/mapOfScan2map", 1
    );
    // Global map visualization
    pub_global_map = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "distributedMapping/globalMap", 1
    );
    // Path for independent robot
    pub_global_path = this->create_publisher<nav_msgs::msg::Path>(
        "distributedMapping/path", 1
    );
    pub_local_path = this->create_publisher<nav_msgs::msg::Path>(
        "distributedMapping/localPath", 1
    );
    // Keypose cloud
    pub_keypose_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "distributedMapping/keyposeCloud", 1
    );

    /*** message information ***/
	cloud_for_descript_ds.reset(new pcl::PointCloud<PointPose3D>()); 

    /*** downsample filter ***/
	downsample_filter_for_descriptor.setLeafSize(descript_leaf_size_, descript_leaf_size_, descript_leaf_size_);
	downsample_filter_for_intra_loop.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
	downsample_filter_for_inter_loop.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
	downsample_filter_for_inter_loop2.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
	downsample_filter_for_inter_loop3.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);

    // mutex
    global_path.poses.clear();
    local_path.poses.clear();

    // distributed loop closure
    inter_robot_loop_ptr = 0;
    intra_robot_loop_ptr = 0;

    intra_robot_loop_close_flag = false;

    // descriptor type default is lidar iris
    // adding an if-statement for further dev of other descriptors
    if(descriptor_type_num_ == DescriptorType::LidarIris){
        keyframe_descriptor = unique_ptr<scan_descriptor>(new lidar_iris_descriptor(
            80, 360, 128, 0.4, 30, 2, 10, 4, 18, 1.6, 0.75, number_of_robots_, robot_id
        ));
        // rows, columns, n_scan, distance_threshold, exclude_Recent_frame_num, match_mode, 
        // knn_candidate_num, nscale, min_wave_length, mult, sigma_on_f, number_of_robots, robot_id
    }

    loop_closures_candidates.clear();
    loop_indexes.clear();

    // radius search
    copy_keyposes_cloud_3d.reset(new pcl::PointCloud<PointPose3D>());
    copy_keyposes_cloud_6d.reset(new pcl::PointCloud<PointPose6D>());
    kdtree_history_keyposes.reset(new pcl::KdTreeFLANN<PointPose3D>());

    // noise model
    odometry_noise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
	prior_noise = noiseModel::Isotropic::Variance(6, 1e-12);

    // isam2 params init
    ISAM2Params isam2_parameters;
    isam2_parameters.relinearizeThreshold = 0.1;
    isam2_parameters.relinearizeSkip = 1;
    isam2 = new ISAM2(isam2_parameters);

    keyposes_cloud_3d.reset(new pcl::PointCloud<PointPose3D>());
	keyposes_cloud_6d.reset(new pcl::PointCloud<PointPose6D>());

    optimizer = boost::shared_ptr<distributed_mapper::DistributedMapper>(
        new distributed_mapper::DistributedMapper(robot_id + 'a')
    );

    steps_of_unchange_graph = 0;

    local_pose_graph = boost::make_shared<NonlinearFactorGraph>();
    initial_values = boost::make_shared<Values>();
	graph_values_vec = make_pair(local_pose_graph, initial_values);

    graph_disconnected = true;

	lowest_id_included = robot_id;
	lowest_id_to_included = lowest_id_included;
	prior_owner = robot_id;
	prior_added = false;

    adjacency_matrix = gtsam::Matrix::Zero(number_of_robots_, number_of_robots_);
	optimization_order.clear();
	in_order = false;

    optimizer_state = OptimizerState::Idle;
	optimization_steps = 0;
	sent_start_optimization_flag = false;

	current_rotation_estimate_iteration = 0;
	current_pose_estimate_iteration = 0;

	latest_change = -1;
	steps_without_change = 0;

	rotation_estimate_start = false;
	pose_estimate_start = false;
	rotation_estimate_finished = false;
	pose_estimate_finished = false;
	estimation_done = false;

	neighboring_robots.clear();
	neighbors_within_communication_range.clear();
	neighbors_started_optimization.clear();
	neighbor_state.clear();

	neighbors_rotation_estimate_finished.clear();
	neighbors_pose_estimate_finished.clear();
	neighbors_estimation_done.clear();

	neighbors_lowest_id_included.clear();
	neighbors_anchor_offset.clear();

    local_pose_graph_no_filtering = boost::make_shared<NonlinearFactorGraph>();  

    pose_estimates_from_neighbors.clear();
	other_robot_keys_for_optimization.clear();

	accepted_keys.clear();
	rejected_keys.clear();
	measurements_rejected_num = 0;
	measurements_accepted_num = 0;
	
	optimizer->setUseBetweenNoiseFlag(use_between_noise_); // use between noise or not in optimizePoses
	optimizer->setUseLandmarksFlag(use_landmarks_); // use landmarks
	optimizer->loadSubgraphAndCreateSubgraphEdge(graph_values_vec); // load subgraphs
	optimizer->setVerbosity(distributed_mapper::DistributedMapper::ERROR); // verbosity level
	optimizer->setFlaggedInit(use_flagged_init_);
	optimizer->setUpdateType(distributed_mapper::DistributedMapper::incUpdate);
	optimizer->setGamma(gamma_);

	/*** Initialize Timer ***/
    if (global_optimization_enable_) {
        distributed_mapping_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(mapping_process_interval_),
            std::bind(&distributedMapping::run, this)
        );
    }

	RCLCPP_INFO(logger, "distributed mapping class initialization finish");
}

distributedMapping::~distributedMapping(){}

pcl::PointCloud<PointPose3D>::Ptr distributedMapping::getLocalKeyposesCloud3D()
{
	return keyposes_cloud_3d;
}

pcl::PointCloud<PointPose6D>::Ptr distributedMapping::getLocalKeyposesCloud6D()
{
	return keyposes_cloud_6d;
}

pcl::PointCloud<PointPose3D> distributedMapping::getLocalKeyframe(const int& index)
{
	return robots[robot_id].keyframe_cloud_array[index];
}

Pose3 distributedMapping::getLatestEstimate()
{
	return isam2_keypose_estimate;
}

void distributedMapping::poseCovariance2msg(
    const graph_utils::PoseWithCovariance& pose,
    geometry_msgs::msg::PoseWithCovariance& msg) {

    // Set position
    msg.pose.position.x = pose.pose.x();
    msg.pose.position.y = pose.pose.y();
    msg.pose.position.z = pose.pose.z();

    // Set orientation
    Eigen::Quaterniond quaternion = pose.pose.rotation().toQuaternion();
    msg.pose.orientation.w = quaternion.w();
    msg.pose.orientation.x = quaternion.x();
    msg.pose.orientation.y = quaternion.y();
    msg.pose.orientation.z = quaternion.z();

    // Set covariance
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            msg.covariance[i * 6 + j] = pose.covariance_matrix(i, j);
        }
    }
}

void distributedMapping::msg2poseCovariance(
    const geometry_msgs::msg::PoseWithCovariance& msg,
    graph_utils::PoseWithCovariance& pose) {

    // Convert orientation to Rot3
    Rot3 rotation(msg.pose.orientation.w, msg.pose.orientation.x,
                         msg.pose.orientation.y, msg.pose.orientation.z);

    // Convert position to Point3
    Point3 translation(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

    // Set the pose using Pose3
    pose.pose = Pose3(rotation, translation);

    // Initialize covariance matrix to zeros
    pose.covariance_matrix = gtsam::Matrix::Zero(6, 6);

    // Fill covariance matrix
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            pose.covariance_matrix(i, j) = msg.covariance[i * 6 + j];
        }
    }
}