#include "multiAgentMapping/distributed_mapping/distributedMapping.hpp"

#include <rclcpp/rclcpp.hpp>
#include <filesystem>  // For directory checking (C++17)

distributedMapping::distributedMapping() : paramsServer() {
    std::string log_name = robot_name + "_distributed_mapping";

    // Log directory initialization
    std::string log_dir = "/log";
    if (!std::filesystem::exists(log_dir)) {
        std::filesystem::create_directory(log_dir);
        RCLCPP_INFO(this->get_logger(), "Created log directory: %s", log_dir.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Using existing log directory: %s", log_dir.c_str());
    }

    // Log message (replacing Google Logging)
    RCLCPP_INFO(this->get_logger(), "Distributed mapping initialized with log name: %s", log_name.c_str());
}

distributedMapping::~distributedMapping(){}