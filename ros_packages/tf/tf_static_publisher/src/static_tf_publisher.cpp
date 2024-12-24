#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>

using json = nlohmann::json;

class StaticTFPublisher : public rclcpp::Node
{
public:
    StaticTFPublisher() : Node("static_tf_publisher")
    {
        // Use fixed absolute path for JSON file
        std::string json_file = "/home/regastation/workspaces/masters_ws/src/MultiAgentMapping/initial_positions.json";

        // Load and publish static transforms
        load_and_publish(json_file);
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;

    void load_and_publish(const std::string &json_file)
    {
        // Load JSON file
        std::ifstream file(json_file);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open JSON file: %s", json_file.c_str());
            return;
        }

        json robots;
        file >> robots;

        // Initialize static transform broadcaster
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Publish static transforms for each robot
        for (auto &robot : robots.items())
        {
            std::string robot_name = robot.key();
            std::string namespace_ = robot.value()["namespace"];

            // Read position
            double x = robot.value()["position"]["x"];
            double y = robot.value()["position"]["y"];
            double z = robot.value()["position"]["z"];

            // Read rotation
            double qx = robot.value()["rotation"]["x"];
            double qy = robot.value()["rotation"]["y"];
            double qz = robot.value()["rotation"]["z"];
            double qw = robot.value()["rotation"]["w"];

            // Publish transform
            publish_static_tf(namespace_, x, y, z, qx, qy, qz, qw);
        }
    }

    void publish_static_tf(const std::string &ns,
                           double x, double y, double z,
                           double qx, double qy, double qz, double qw)
    {
        geometry_msgs::msg::TransformStamped t;


        // Use ROS Sim time
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = ns + "/odom"; // Use namespace in frame_id

        // Set position
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;

        // Set rotation (quaternion)
        t.transform.rotation.x = qx;
        t.transform.rotation.y = qy;
        t.transform.rotation.z = qz;
        t.transform.rotation.w = qw;

        broadcaster_->sendTransform(t);
        RCLCPP_INFO(this->get_logger(), "Published static transform: world -> %s/odom", ns.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticTFPublisher>());
    rclcpp::shutdown();
    return 0;
}
