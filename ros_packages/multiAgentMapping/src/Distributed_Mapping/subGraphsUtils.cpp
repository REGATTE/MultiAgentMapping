#include "multiAgentMapping/Distributed_Mapping/subGraphsUtils.hpp"
#include "multiAgentMapping/LIO_SAM/utility.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>

subGraphMapping::subGraphMapping() : rclcpp::Node("sub_graph_mapping") {
    std::string robot_name = this->get_namespace();
    if(robot_name.length() < 1){
            RCLCPP_ERROR(this->get_logger(), "Invalid robot prefix (should be longer than a letter): %s", robot_name.c_str());
            rclcpp::shutdown();
        }
    robot_namespace = robot_name.substr(1); // Extract the "/"

    // Ensure last character of the name is a digit
    if(!std::isdigit(robot_namespace.back())){
        RCLCPP_ERROR(this->get_logger(), "Invalid namespace format: last character is not a digit!");
        rclcpp::shutdown();
    }
    // extract last char, convert to int and assign as robot_id
    robot_id = robot_namespace.back() - '0';
    RCLCPP_INFO(this->get_logger(), "Robot ID is %d", robot_id); 
}

void subGraphMapping::performDistributedMapping(
    const Pose3& pose_to,
    const pcl::PointCloud<PointPose3D>::Ptr& frame_to,
    const rclcpp::Time& timestamp) {
        RCLCPP_INFO(this->get_logger(), "Performing distributed mapping for robot id: %d", robot_id);
    }