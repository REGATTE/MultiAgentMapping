#include "multiAgentMapping/distributed_mapping/distributedMapping.hpp"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
    class distributedMapping: constructor and destructor
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
distributedMapping::distributedMapping() : paramsServer()
{
    // Create log name by appending to name_
    std::string log_name = robot_name + "_distributed_mapping";

    // Get logger instance
    auto logger = rclcpp::get_logger(log_name);

    // Get log directory
    std::string log_dir = std::string(std::getenv("HOME")) + "/mutliAgentMapping_log";

    // Log initialization message
    RCLCPP_INFO(logger, "Distributed mapping class initialization. Log name: %s, Log directory: %s",  log_name.c_str(), log_dir.c_str());

    std::string robot_namespace_ = this->get_namespace();
    if(robot_namespace_.length() < 1){
        RCLCPP_ERROR(this->get_logger(), "[paramsServer] -> Invalid robot namespace");
        rclcpp::shutdown();
    }
    std::string robot_name_ = robot_namespace_.substr(1); // remove the "/"

    // robot name <-> id mapping
    std::map<std::string, int> robot_map = {
        {"scout_1_1", 1},
        {"scout_2_2", 2},
        {"scout_3_3", 3}
    };

    // Check if the derived robot name exists in the map
    if (robot_map.find(robot_name_) == robot_map.end()) {
        RCLCPP_ERROR(logger, "Robot name '%s' not found in the robot map.", robot_name_.c_str());
        rclcpp::shutdown();
        return;
    }

    // Retrieve the robot ID from the map
    int robot_id_ = robot_map[robot_name_];
    RCLCPP_INFO(logger, "Found robot %s with ID %d", robot_name_.c_str(), robot_id_);

    singleRobot robot; // define each robot struct
    /*** Multi-robot setup ***/
    if (number_of_robots_ < 1) {
        RCLCPP_ERROR(logger, "Number of robots must be greater than 0.");
        rclcpp::shutdown();
        return;
    }

    for (const auto& [robot_name, robot_id]: robot_map){
        robot.robot_id = robot_id;
        robot.robot_name = robot_name;

        robot.odom_frame_ = odom_frame_;

        RCLCPP_INFO(this->get_logger(), "Initialising robot %s with ID %d", robot_name.c_str(), robot_id);

        // this robot's publisher and subscriber
        if(robot_id == robot_id_){
            RCLCPP_INFO(this->get_logger(), "This robot %s and ID %d", robot_name.c_str(), robot_id);
            if(intra_robot_loop_closure_enable_ || inter_robot_loop_closure_enable_){
                // publish global descriptors
                robot.pub_descriptors = this->create_publisher<multi_agent_mapping::msg::GlobalDescriptor>(
                    robot_name + "/distributedMapping/globalDescriptors", 5
                );
                // publish Loop Info
                robot.pub_loop_info = this->create_publisher<multi_agent_mapping::msg::LoopInfo>(
                    robot_name + "/distributedMapping/loopInfo", 5
                );
            }
            
            if(global_optimization_enable_){
                // publish optimization state
                robot.pub_optimization_state = this->create_publisher<std_msgs::msg::Int8>(
                    robot_name + "/distributedMapping/optimizationState", 50
                );
                robot.pub_rotation_estimate_state = this->create_publisher<std_msgs::msg::Int8>(
                    robot_name + "/distributedMapping/rotationEstimateState", 50
                    );
                robot.pub_pose_estimate_state = this->create_publisher<std_msgs::msg::Int8>(
                    robot_name + "/distributedMapping/poseEstimateState", 50
                    );
                robot.pub_neighbor_rotation_estimates = this->create_publisher<multi_agent_mapping::msg::NeighborEstimate>(
                    robot_name + "/distributedMapping/neighborRotationEstimates", 50
                    );
                robot.pub_neighbor_pose_estimates = this->create_publisher<multi_agent_mapping::msg::NeighborEstimate>(
                    robot_name + "/distributedMapping/neighborPoseEstimates", 50
                    );
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Other robot %s and ID %d", robot_name.c_str(), robot_id);
            if (intra_robot_loop_closure_enable_ || inter_robot_loop_closure_enable_) {
                robot.sub_descriptors = this->create_subscription<multi_agent_mapping::msg::GlobalDescriptor>(
                    robot_name + "/distributedMapping/globalDescriptors", 50,
                    std::bind(&distributedMapping::globalDescriptorHandler, this, std::placeholders::_1, robot_id)
                    );
                robot.sub_loop_info = this->create_subscription<multi_agent_mapping::msg::LoopInfo>(
                    robot_name + "/distributedMapping/loopInfo", 50,
                    std::bind(&distributedMapping::loopInfoHandler, this, std::placeholders::_1, robot_id)
                    );
            }
            if (global_optimization_enable_) {
                robot.sub_optimization_state = this->create_subscription<std_msgs::msg::Int8>(
                    robot_name + "/distributedMapping/optimizationState", 50,
                    std::bind(&distributedMapping::optStateHandler, this, std::placeholders::_1, robot_id)
                    );
                robot.sub_rotation_estimate_state = this->create_subscription<std_msgs::msg::Int8>(
                    robot_name + "/distributedMapping/rotationEstimateState", 50,
                    std::bind(&distributedMapping::rotationStateHandler, this, std::placeholders::_1, robot_id)
                    );
                robot.sub_pose_estimate_state = this->create_subscription<std_msgs::msg::Int8>(
                    robot_name + "/distributedMapping/poseEstimateState", 50,
                    std::bind(&distributedMapping::poseStateHandler, this, std::placeholders::_1, robot_id)
                    );
                robot.sub_neighbor_rotation_estimates = this->create_subscription<multi_agent_mapping::msg::NeighborEstimate>(
                    robot_name + "/distributedMapping/neighborRotationEstimates", 50,
                    std::bind(&distributedMapping::neighborRotationHandler, this, std::placeholders::_1, robot_id)
                    );
                robot.sub_neighbor_pose_estimates = this->create_subscription<multi_agent_mapping::msg::NeighborEstimate>(
                    robot_name + "/distributedMapping/neighborPoseEstimates", 50,
                    std::bind(&distributedMapping::neighborPoseHandler, this, std::placeholders::_1, robot_id)
                    );
            }
        }

        // other
        robot.time_cloud_input_stamp = this->now();
        robot.time_cloud_input = 0.0;
        robot.keyframe_cloud = std::make_shared<pcl::PointCloud<PointPose3D>>();
        robot.keyframe_cloud_array.clear();

        // add the robots to the list of robots
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
	cloud_for_decript_ds.reset(new pcl::PointCloud<PointPose3D>()); 

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
        // knn_candidate_num, nscale, min_wave_length, mult, sigma_on_f, number_of_robots, robot_f
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


}
