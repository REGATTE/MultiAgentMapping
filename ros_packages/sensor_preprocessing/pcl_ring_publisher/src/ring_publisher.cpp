// C++ Node: ring_publisher.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class RingPublisher : public rclcpp::Node
{
public:
    RingPublisher() : Node("ring_publisher")
    {
        // Declare parameters
        this->declare_parameter<std::string>("robot_namespace", "robot_x");
        this->declare_parameter<std::string>("input_topic", "scan3D");

        // Get parameters
        std::string robot_namespace = this->get_parameter("robot_namespace").as_string();
        std::string input_topic = this->get_parameter("input_topic").as_string();

        // Full topic name with namespace
        std::string topic_name = "/" + robot_namespace + "/" + input_topic;

        // Create Subscriber and Publisher
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, 10, std::bind(&RingPublisher::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/" + robot_namespace + "/scan3D_with_rings", 10);
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto output = *msg;

        // Add a 'ring' field if it doesn't exist
        bool has_ring = false;
        for (const auto &field : msg->fields)
        {
            if (field.name == "ring")
            {
                has_ring = true;
                break;
            }
        }

        if (!has_ring)
        {
            // Add 'ring' field to the point cloud
            sensor_msgs::PointCloud2Modifier modifier(output);
            modifier.setPointCloud2Fields(4, 
                                          "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                          "ring", 1, sensor_msgs::msg::PointField::UINT16);

            sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(output, "ring");

            int ring_id = 0;
            for (; iter_ring != iter_ring.end(); ++iter_ring)
            {
                *iter_ring = ring_id % 128; // 128 beams
                ring_id++;
            }
        }

        pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RingPublisher>());
    rclcpp::shutdown();
    return 0;
}
