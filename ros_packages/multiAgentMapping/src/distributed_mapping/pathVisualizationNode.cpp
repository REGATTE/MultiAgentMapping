#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <multi_agent_mapping/msg/loop_info.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

using namespace std;

class MultiRobotVisualizationNode : public rclcpp::Node {
public:
    MultiRobotVisualizationNode() : Node("multi_robot_visualization_node") {
        this->declare_parameter<int>("number_of_robots", 2);
        number_of_robots_ = this->get_parameter("number_of_robots").as_int();
        
        pub_aligned_paths_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization/aligned_robot_paths", 10);
        
        initial_transforms_set_.resize(number_of_robots_, false);
        initial_orientations_.resize(number_of_robots_);
        initial_transforms_.resize(number_of_robots_);
        transform_history_.resize(number_of_robots_);
        robot_directions_.resize(number_of_robots_);
        original_directions_.resize(number_of_robots_);
        transform_point_.resize(number_of_robots_);
        last_loop_closure_time_ = this->now();

        for (int i = 0; i < number_of_robots_; i++) {
            std::string robot_name = "a";
            robot_name[0] += i;
            
            robot_paths_.push_back(nav_msgs::msg::Path());
            relative_transforms_.push_back(gtsam::Pose3());
            robot_directions_[i] = gtsam::Point3(0, 0, 0);
            
            auto path_callback = [this, i](const nav_msgs::msg::Path::SharedPtr msg) {
                pathCallback(msg, i);
            };
            
            auto loop_callback = [this, i](const multi_agent_mapping::msg::LoopInfo::SharedPtr msg) {
                loopClosureCallback(msg, i);
            };

            path_subscribers_.push_back(
                this->create_subscription<nav_msgs::msg::Path>(
                    robot_name + "/distributedMapping/path", 10, path_callback));
                    
            loop_subscribers_.push_back(
                this->create_subscription<multi_agent_mapping::msg::LoopInfo>(
                    robot_name + "/distributedMapping/loopInfo", 10, loop_callback));
        }

        visualization_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MultiRobotVisualizationNode::visualizationCallback, this));
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg, int robot_id) {
        robot_paths_[robot_id] = *msg;
        
        if (msg->poses.size() >= 2) {
            auto& last_pose = msg->poses.back().pose.position;
            auto& prev_pose = msg->poses[msg->poses.size()-2].pose.position;
            
            robot_directions_[robot_id] = gtsam::Point3(
                last_pose.x - prev_pose.x,
                last_pose.y - prev_pose.y,
                last_pose.z - prev_pose.z
            ).normalized();
        }
    }

    void loopClosureCallback(const multi_agent_mapping::msg::LoopInfo::SharedPtr msg, int id) {
        if ((int)msg->noise != 999 && (int)msg->noise != 888) {
            if (msg->robot1 != 0) {
                gtsam::Pose3 pose0 = gtsam::Pose3(
                    gtsam::Rot3::Quaternion(
                        msg->pose0.rotation.w,
                        msg->pose0.rotation.x,
                        msg->pose0.rotation.y,
                        msg->pose0.rotation.z),
                    gtsam::Point3(
                        msg->pose0.translation.x,
                        msg->pose0.translation.y,
                        msg->pose0.translation.z));

                gtsam::Pose3 pose1 = gtsam::Pose3(
                    gtsam::Rot3::Quaternion(
                        msg->pose1.rotation.w,
                        msg->pose1.rotation.x,
                        msg->pose1.rotation.y,
                        msg->pose1.rotation.z),
                    gtsam::Point3(
                        msg->pose1.translation.x,
                        msg->pose1.translation.y,
                        msg->pose1.translation.z));

                if (!initial_transforms_set_[msg->robot1]) {
                    initial_transforms_set_[msg->robot1] = true;
                    
                    // Store the intersection point (loop closure point)
                    transform_point_[msg->robot1] = gtsam::Point3(
                        msg->pose1.translation.x,
                        msg->pose1.translation.y,
                        msg->pose1.translation.z
                    );

                    // Store the original directions at loop closure
                    original_directions_[msg->robot1] = robot_directions_[msg->robot1];
                    initial_orientations_[msg->robot1] = pose1.rotation();
                }

                // Calculate relative transform between poses at loop closure point
                gtsam::Pose3 relative_transform = pose0.between(pose1);
                relative_transforms_[msg->robot1] = relative_transform;
                transform_history_[msg->robot1].push_back(relative_transform);
            }
        }
    }

    void visualizationCallback() {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (int i = 0; i < number_of_robots_; i++) {
            if (robot_paths_[i].poses.empty()) continue;

            visualization_msgs::msg::Marker path_marker;
            path_marker.header.frame_id = "world";
            path_marker.header.stamp = this->now();
            path_marker.ns = "robot_path_" + std::to_string(i);
            path_marker.id = i;
            path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            path_marker.action = visualization_msgs::msg::Marker::ADD;
            path_marker.pose.orientation.w = 1.0;
            path_marker.scale.x = 0.1;
            
            path_marker.color.a = 1.0;
            path_marker.color.r = (i == 0) ? 1.0 : 0.0;
            path_marker.color.g = (i == 1) ? 1.0 : 0.0;
            path_marker.color.b = 0.0;

            for (const auto& pose : robot_paths_[i].poses) {
                geometry_msgs::msg::Point p = pose.pose.position;
                
                if (i != 0 && initial_transforms_set_[i] && !transform_history_[i].empty()) {
                    gtsam::Point3 point(p.x, p.y, p.z);
                    gtsam::Point3 intersection_point = transform_point_[i];
                    
                    // Transform to intersection point's frame
                    point = point - intersection_point;
                    
                    // Apply the relative transform from loop closure
                    gtsam::Pose3 current_transform = transform_history_[i].back();
                    point = current_transform.transformFrom(point);
                    
                    // Calculate relative orientation between robots at loop closure
                    if (!original_directions_[i].isZero() && !robot_directions_[0].isZero()) {
                        // Get the direction vectors at loop closure time
                        gtsam::Point3 orig_dir = original_directions_[i];
                        gtsam::Point3 ref_dir = robot_directions_[0];
                        
                        // Calculate angles
                        double orig_angle = atan2(orig_dir.y(), orig_dir.x());
                        double ref_angle = atan2(ref_dir.y(), ref_dir.x());
                        
                        // Calculate rotation needed to align directions
                        double rotation_angle = ref_angle - orig_angle + M_PI;  // Add PI to flip direction
                        
                        // Normalize angle to [-π, π]
                        while (rotation_angle > M_PI) rotation_angle -= 2 * M_PI;
                        while (rotation_angle < -M_PI) rotation_angle += 2 * M_PI;
                        
                        gtsam::Rot3 direction_correction = gtsam::Rot3::Rz(rotation_angle);
                        point = direction_correction.rotate(point);
                    }
                    
                    // Transform back to global frame
                    point = point + intersection_point;
                    
                    p.x = point.x();
                    p.y = point.y();
                    p.z = point.z();
                }
                
                path_marker.points.push_back(p);
            }
            
            marker_array.markers.push_back(path_marker);
        }
        
        pub_aligned_paths_->publish(marker_array);
    }

    int number_of_robots_;
    std::vector<nav_msgs::msg::Path> robot_paths_;
    std::vector<gtsam::Pose3> relative_transforms_;
    std::vector<gtsam::Pose3> initial_transforms_;
    std::vector<std::vector<gtsam::Pose3>> transform_history_;
    std::vector<bool> initial_transforms_set_;
    std::vector<gtsam::Rot3> initial_orientations_;
    std::vector<gtsam::Point3> robot_directions_;
    std::vector<gtsam::Point3> original_directions_;
    std::vector<gtsam::Point3> transform_point_;
    rclcpp::Time last_loop_closure_time_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> path_subscribers_;
    std::vector<rclcpp::Subscription<multi_agent_mapping::msg::LoopInfo>::SharedPtr> loop_subscribers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_aligned_paths_;
    rclcpp::TimerBase::SharedPtr visualization_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiRobotVisualizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}