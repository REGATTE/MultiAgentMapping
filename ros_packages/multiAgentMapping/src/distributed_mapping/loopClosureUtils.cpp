#include "multiAgentMapping/distributed_mapping/distributedMapping.hpp"

void distributedMapping::loopInfoHandler(
    const multi_agent_mapping::msg::LoopInfo::SharedPtr& msg,
    int id
){
    // Situation 1: need to add pointcloud for loop closure verification
    if((int)msg->noise == 999){
        if(msg->robot0 != robot_id){
            return;
        }

        //copy message
        multi_agent_mapping::msg::LoopInfo loop_msg;
        loop_msg.robot0 = msg->robot0;
        loop_msg.robot1 = msg->robot1;
        loop_msg.index0 = msg->index0;
		loop_msg.index1 = msg->index1;
		loop_msg.init_yaw = msg->init_yaw;
		loop_msg.noise = 888.0; // this loop need verification

        // CHECK_LT(loop_msg.index0,keyposes_cloud_6d->size());
		// CHECK_LT(loop_msg.index0,robots[robot_id].keyframe_cloud_array.size());
        if (loop_msg.index0 >= keyposes_cloud_6d->size()) {
            RCLCPP_ERROR(this->get_logger(), "[LoopClosureUtils : loopInfoHandler] -> Index out of bounds: loop_msg.index0=%d, size=%lu", 
                        loop_msg.index0, keyposes_cloud_6d->size());
            throw std::out_of_range("[LoopClosureUtils : loopInfoHandler] -> loop_msg.index0 is out of bounds of keyposes_cloud_6d.");
        }

        if (loop_msg.index0 >= robots[robot_id].keyframe_cloud_array.size()) {
            RCLCPP_ERROR(this->get_logger(), "[LoopClosureUtils : loopInfoHandler] -> Index out of bounds: loop_msg.index0=%d, keyframe_cloud_array size=%lu", 
                        loop_msg.index0, robots[robot_id].keyframe_cloud_array.size());
            throw std::out_of_range("[LoopClosureUtils : loopInfoHandler] -> loop_msg.index0 is out of bounds of keyframe_cloud_array.");
        }

        // filtered pointcloud
		pcl::PointCloud<PointPose3D>::Ptr cloudTemp(new pcl::PointCloud<PointPose3D>());
		*cloudTemp = robots[robot_id].keyframe_cloud_array[loop_msg.index0];
		downsample_filter_for_inter_loop2.setInputCloud(cloudTemp);
		downsample_filter_for_inter_loop2.filter(*cloudTemp);
		pcl::toROSMsg(*cloudTemp, loop_msg.scan_cloud);
		// relative pose
		loop_msg.pose0 = gtsamPoseToTransform(pclPointTogtsamPose3(keyposes_cloud_6d->points[loop_msg.index0]));

		// publish to others for verification
		robots[robot_id].pub_loop_info->publish(loop_msg);
    }

    // Situation 2: need to verify loop closure in this robot
    else if((int)msg->noise == 888){
		if(msg->robot1 != robot_id){
			return;
		}

		RCLCPP_INFO(this->get_logger(), "[LoopClosureUtils : loopInfoHandler] -> [loopInfoHandler(%d)] check loop %d-%d %d-%d.", 
            id, msg->robot0, msg->index0, msg->robot1, msg->index1);

		loop_closures_candidates.push_back(*msg);
	}

    // Situation 3: add verified loop closure
	else
	{
		RCLCPP_INFO(this->get_logger(), "[LoopClosureUtils : loopInfoHandler] -> [loopInfoHandler(%d)] add loop %d-%d %d-%d.", 
            id, msg->robot0, msg->index0, msg->robot1, msg->index1);

		// extract loop
		Vector Vector6(6);
		Vector6 << msg->noise, msg->noise, msg->noise, msg->noise, msg->noise, msg->noise;
		noiseModel::Diagonal::shared_ptr loop_noise = noiseModel::Diagonal::Variances(Vector6);
		Pose3 pose_between = transformToGtsamPose(msg->pose_between);
		NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(
			Symbol('a'+msg->robot0, msg->index0), Symbol('a'+msg->robot1, msg->index1), pose_between, loop_noise));
		
		// update adjacency matrix
		adjacency_matrix(msg->robot0, msg->robot1) += 1;
		adjacency_matrix(msg->robot1, msg->robot0) += 1;
		if(msg->robot0 == robot_id || msg->robot1 == robot_id)
		{
			// add loop factor
			local_pose_graph->add(factor);
			local_pose_graph_no_filtering->add(factor);
			// enable distributed mapping
			sent_start_optimization_flag = true;

			// update pose estimate (for PCM)
			Key key;
			graph_utils::PoseWithCovariance pose;
			pose.covariance_matrix = loop_noise->covariance();
			pose.pose = transformToGtsamPose(msg->pose1);
			key = Symbol('a'+msg->robot1, msg->index1).key();
			updatePoseEstimateFromNeighbor(msg->robot1, key, pose);
			pose.pose = transformToGtsamPose(msg->pose0);
			key = Symbol('a'+msg->robot0, msg->index0).key();
			updatePoseEstimateFromNeighbor(msg->robot0, key, pose);

			// add transform to local map (for PCM)
			auto new_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);
			Matrix covariance_matrix = loop_noise->covariance();
			robot_local_map.addTransform(*new_factor, covariance_matrix);
		}
	}
}

void distributedMapping::calculateTransformation(
    const int& loop_key0,
    const int& loop_key1
){
    if (loop_key0 >= copy_keyposes_cloud_6d->size()) {
        RCLCPP_ERROR(this->get_logger(), "[LoopClosureUtils : calculateTransformation] -> Index out of bounds: loop_key0=%d, copy_keyposes_cloud_6d size=%lu", 
                    loop_key0, copy_keyposes_cloud_6d->size());
        throw std::out_of_range("[LoopClosureUtils : calculateTransformation] -> loop_key0 is out of bounds of copy_keyposes_cloud_6d.");
    }

    // get initial pose
	Pose3 loop_pose0 = pclPointTogtsamPose3(copy_keyposes_cloud_6d->points[loop_key0]);
	Pose3 loop_pose1 = pclPointTogtsamPose3(copy_keyposes_cloud_6d->points[loop_key1]);

    // extract cloud
	pcl::PointCloud<PointPose3D>::Ptr scan_cloud(new pcl::PointCloud<PointPose3D>());
	pcl::PointCloud<PointPose3D>::Ptr scan_cloud_ds(new pcl::PointCloud<PointPose3D>());
	loopFindNearKeyframes(scan_cloud, loop_key0, 0);
	downsample_filter_for_intra_loop.setInputCloud(scan_cloud);
	downsample_filter_for_intra_loop.filter(*scan_cloud_ds);
	pcl::PointCloud<PointPose3D>::Ptr map_cloud(new pcl::PointCloud<PointPose3D>());
	pcl::PointCloud<PointPose3D>::Ptr map_cloud_ds(new pcl::PointCloud<PointPose3D>());
	loopFindNearKeyframes(map_cloud, loop_key1, history_keyframe_search_num_);
	downsample_filter_for_intra_loop.setInputCloud(map_cloud);
	downsample_filter_for_intra_loop.filter(*map_cloud_ds);

    // fail safe check for cloud
	if(scan_cloud->size() < 300 || map_cloud->size() < 1000){
		RCLCPP_ERROR(this->get_logger(), "[LoopClosureUtils : calculateTransformation] -> keyFrameCloud too little points 1");
		return;
	}

    // publish cloud
	if(pub_scan_of_scan2map->get_subscription_count() != 0)
	{
		sensor_msgs::msg::PointCloud2 scan_cloud_msg;
		pcl::toROSMsg(*scan_cloud_ds, scan_cloud_msg);
		scan_cloud_msg.header.stamp = this->now();
		scan_cloud_msg.header.frame_id = world_frame_;
		pub_scan_of_scan2map->publish(scan_cloud_msg);
	}
	if(pub_map_of_scan2map->get_subscription_count() != 0)
	{
		sensor_msgs::msg::PointCloud2 map_cloud_msg;
		pcl::toROSMsg(*map_cloud_ds, map_cloud_msg);
		map_cloud_msg.header.stamp = this->now();
		map_cloud_msg.header.frame_id = world_frame_;
		pub_map_of_scan2map->publish(map_cloud_msg);
	}

    // icp settings
	static pcl::IterativeClosestPoint<PointPose3D, PointPose3D> icp;
	icp.setMaxCorrespondenceDistance(2*search_radius_);
	icp.setMaximumIterations(50);
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setRANSACIterations(0);
	// icp.setRANSACOutlierRejectionThreshold(ransac_outlier_reject_threshold_);

    // align clouds
	icp.setInputSource(scan_cloud_ds);
	icp.setInputTarget(map_cloud_ds);
	pcl::PointCloud<PointPose3D>::Ptr unused_result(new pcl::PointCloud<PointPose3D>());
	icp.align(*unused_result);

    // Check if pass ICP fitness score
    float fitness_score = icp.getFitnessScore();
    if (icp.hasConverged() == false || fitness_score > fitness_score_threshold_) {
        RCLCPP_DEBUG(this->get_logger(), "[LoopClosureUtils : calculateTransformation] -> \033[1;34m[IntraLoop<%d>] [%d]-[%d] ICP failed (%.2f > %.2f). Reject.\033[0m",
                    robot_id, loop_key0, loop_key1, fitness_score, fitness_score_threshold_);
        RCLCPP_INFO(this->get_logger(), "[LoopClosureUtils : calculateTransformation] -> [IntraLoop<%d>] ICP failed (%.2f > %.2f). Reject.", 
                    robot_id, fitness_score, fitness_score_threshold_);
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "[LoopClosureUtils : calculateTransformation] -> \033[1;34m[IntraLoop<%d>] [%d]-[%d] ICP passed (%.2f < %.2f). Add.\033[0m", 
                robot_id, loop_key0, loop_key1, fitness_score, fitness_score_threshold_);
    RCLCPP_INFO(this->get_logger(), "[LoopClosureUtils : calculateTransformation] -> [IntraLoop<%d>] ICP passed (%.2f < %.2f). Add.", 
                robot_id, fitness_score, fitness_score_threshold_);

    // get pose transformation
	float x, y, z, roll, pitch, yaw;
	Eigen::Affine3f icp_final_tf;
	icp_final_tf = icp.getFinalTransformation();
	pcl::getTranslationAndEulerAngles(icp_final_tf, x, y, z, roll, pitch, yaw);
	Eigen::Affine3f origin_tf = gtsamPoseToAffine3f(loop_pose0);
	Eigen::Affine3f correct_tf = icp_final_tf * origin_tf;
	pcl::getTranslationAndEulerAngles(correct_tf, x, y, z, roll, pitch, yaw);
	Pose3 pose_from = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
	Pose3 pose_to = loop_pose1;
	Pose3 pose_between = pose_from.between(pose_to);
	RCLCPP_INFO(this->get_logger(), "[LoopClosureUtils : calculateTransformation] -> [IntraLoop<%d>] pose_between: %.2f %.2f %.2f", 
            robot_id, pose_between.translation().x(), pose_between.translation().y(), pose_between.translation().z());

    // add loop factor
	Vector vector6(6);
	vector6 << fitness_score, fitness_score, fitness_score, fitness_score, fitness_score, fitness_score;
	noiseModel::Diagonal::shared_ptr loop_noise = noiseModel::Diagonal::Variances(vector6);
	NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(
		Symbol('a'+robot_id, loop_key0), Symbol('a'+robot_id, loop_key1), pose_between, loop_noise));
	isam2_graph.add(factor);
	local_pose_graph->add(factor);
	local_pose_graph_no_filtering->add(factor);
	sent_start_optimization_flag = true; // enable distributed mapping
	intra_robot_loop_close_flag = true;

    // save loop factor in local map (for PCM)
	auto new_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);
	Matrix covariance = loop_noise->covariance();
	robot_local_map.addTransform(*new_factor, covariance);

	auto it = loop_indexs.find(loop_key0);
	if(it == loop_indexs.end() || (it != loop_indexs.end() && it->second != loop_key1)){
		loop_indexs[loop_key0] = loop_key1;
	}
}

void distributedMapping::loopFindGlobalNearKeyframes(
	pcl::PointCloud<PointPose3D>::Ptr& near_keyframes,
	const int& key,
	const int& search_num)
{
	// extract near keyframes
	near_keyframes->clear();
	int pose_num = initial_values->size();
	if (pose_num >= robots[robot_id].keyframe_cloud_array.size()) {
        RCLCPP_ERROR(this->get_logger(), 
             "[LoopClosureUtils : loopFindGlobalNearKeyframe] -> Index out of bounds: pose_num=%d, keyframe_cloud_array size=%zu for robot_id=%d", 
             pose_num, robots[robot_id].keyframe_cloud_array.size(), robot_id);
        throw std::out_of_range("[LoopClosureUtils : loopFindGlobalNearKeyframe] -> pose_num is out of bounds of keyframe_cloud_array.");
    }
	int add_num = 0;
	for(int i = -search_num; i <= search_num*2; ++i){
		if(add_num >= search_num*2){
			break;
		}

		int key_near = key + i;
		if(key_near < 0 || key_near >= pose_num){
			continue;
		}
		
		*near_keyframes += *transformPointCloud(robots[robot_id].keyframe_cloud_array[key_near],
			initial_values->at<Pose3>(Symbol('a'+robot_id, key_near)));
		add_num++;
	}

	if(near_keyframes->empty()){
		return;
	}
}


void distributedMapping::loopFindNearKeyframes(
	pcl::PointCloud<PointPose3D>::Ptr& near_keyframes,
	const int& key,
	const int& search_num)
{
	// extract near keyframes
	near_keyframes->clear();
	int pose_num = copy_keyposes_cloud_6d->size();
	if (pose_num >= robots[robot_id].keyframe_cloud_array.size()) {
        RCLCPP_ERROR(this->get_logger(), 
             "[LoopClosureUtils : loopFindNearKeyframe] -> Index out of bounds: pose_num=%d, keyframe_cloud_array size=%zu for robot_id=%d", 
             pose_num, robots[robot_id].keyframe_cloud_array.size(), robot_id);
        throw std::out_of_range("[LoopClosureUtils : loopFindNearKeyframe] -> pose_num is out of bounds of keyframe_cloud_array.");
    }

	for(int i = -search_num; i <= search_num; ++i){
		int key_near = key + i;
		if(key_near < 0 || key_near >= pose_num){
			continue;
		}
		*near_keyframes += *transformPointCloud(
			robots[robot_id].keyframe_cloud_array[key_near], &copy_keyposes_cloud_6d->points[key_near]);
	}

	if(near_keyframes->empty()){
		return;
	}
}

void distributedMapping::updatePoseEstimateFromNeighbor(
	const int& rid,
	const Key& key,
	const graph_utils::PoseWithCovariance& pose)
{
	graph_utils::TrajectoryPose trajectory_pose;
	trajectory_pose.id = key;
	trajectory_pose.pose = pose;
	// find trajectory
	if(pose_estimates_from_neighbors.find(rid) != pose_estimates_from_neighbors.end()){
		// update pose
		if(pose_estimates_from_neighbors.at(rid).trajectory_poses.find(key) != 
			pose_estimates_from_neighbors.at(rid).trajectory_poses.end()){
			pose_estimates_from_neighbors.at(rid).trajectory_poses.at(key) = trajectory_pose;
		}
		// new pose
		else {
			pose_estimates_from_neighbors.at(rid).trajectory_poses.insert(make_pair(key, trajectory_pose));
			if(key < pose_estimates_from_neighbors.at(rid).start_id)
			{
				pose_estimates_from_neighbors.at(rid).start_id = key;
			}
			if(key > pose_estimates_from_neighbors.at(rid).end_id)
			{
				pose_estimates_from_neighbors.at(rid).end_id = key;
			}
		}
	}
	// insert new trajectory
	else {
		graph_utils::Trajectory new_trajectory;
		new_trajectory.trajectory_poses.insert(make_pair(key, trajectory_pose));
		new_trajectory.start_id = key;
		new_trajectory.end_id = key;
		pose_estimates_from_neighbors.insert(make_pair(rid, new_trajectory));
	}
}