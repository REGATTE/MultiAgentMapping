#include "multiAgentMapping/distributed_mapping/distributedMapping.hpp"
#include <rclcpp/rclcpp.hpp>  // For ROS 2 logging

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
            
        }
    }
}
