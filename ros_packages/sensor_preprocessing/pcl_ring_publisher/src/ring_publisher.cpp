/**
 * @file lidar_ring_converter.cpp
 * @brief ROS 2 Node for converting LiDAR point cloud data to include ring and time fields.
 *
 * This node dynamically subscribes to a specified topic for incoming point cloud data,
 * processes the data to calculate ring and time values for each point, and publishes
 * the updated data to a new topic. It supports dynamic configuration of LiDAR parameters
 * like the number of vertical beams (N_SCAN) and horizontal resolution (Horizon_SCAN).
 *
 * @author [Your Name]
 * @date [Date]
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Global variables for topic names
std::string lidar_topic; 
std::string output_topic;

/**
 * @struct PointXYZIRT
 * @brief Custom point type to store x, y, z, intensity, ring, and time fields.
 */
struct PointXYZIRT
{
    PCL_ADD_POINT4D                  ///< 3D point coordinates (x, y, z)
    PCL_ADD_INTENSITY                ///< Intensity value
    uint16_t ring;                   ///< Ring number
    float time;                      ///< Time offset
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Register custom point type
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

/**
 * @class LidarRingConverter
 * @brief ROS 2 Node for processing and augmenting LiDAR point cloud data.
 */
class LidarRingConverter : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the LidarRingConverter class.
     */
    LidarRingConverter() : Node("lidar_ring_converter")
    {
        // Declare parameters
        this->declare_parameter<std::string>("robot_namespace", "scout_1");
        this->declare_parameter<int>("N_SCAN", 128);
        this->declare_parameter<int>("Horizon_SCAN", 2048);

        // Get parameters
        std::string robot_namespace = this->get_parameter("robot_namespace").as_string();
        N_SCAN = this->get_parameter("N_SCAN").as_int();
        Horizon_SCAN = this->get_parameter("Horizon_SCAN").as_int();

        // Define topic names based on namespace
        lidar_topic = "/" + robot_namespace + "/scan3D";
        output_topic = "/" + robot_namespace + "/scan3D_with_rings";

        // Create subscriber and publisher
        subPC_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, 10, std::bind(&LidarRingConverter::lidarHandle, this, std::placeholders::_1));

        pubPC_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    }

private:
    int N_SCAN;                      ///< Number of vertical beams
    int Horizon_SCAN;                ///< Horizontal resolution
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPC_; ///< Subscriber
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPC_;   ///< Publisher

    /**
     * @brief Publishes the modified point cloud data.
     * @param new_pc Processed point cloud.
     * @param old_msg Original point cloud message header.
     */
    template<typename T>
    void publishPoints(T &new_pc, const sensor_msgs::msg::PointCloud2 &old_msg) {
        new_pc->is_dense = true;

        // Convert to ROS 2 message
        sensor_msgs::msg::PointCloud2 pc_new_msg;
        pcl::toROSMsg(*new_pc, pc_new_msg);
        pc_new_msg.header = old_msg.header;
        pc_new_msg.header.frame_id = "LiDAR"; // Set frame ID
        pubPC_->publish(pc_new_msg);
    }

    /**
     * @brief Callback function to process incoming point cloud data.
     * @param pc_msg Pointer to incoming point cloud message.
     */
    void lidarHandle(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
        // Convert incoming data
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<PointXYZIRT>::Ptr pc_new(new pcl::PointCloud<PointXYZIRT>());
        pcl::fromROSMsg(*pc_msg, *pc);

        // Process each point
        for (size_t point_id = 0; point_id < pc->points.size(); ++point_id) {

            PointXYZIRT new_point;
            new_point.x = pc->points[point_id].x;
            new_point.y = pc->points[point_id].y;
            new_point.z = pc->points[point_id].z;
            new_point.intensity = 0; 

            // Calculate ring ID
            float ang_bottom = 15.0 + 0.1;
            float ang_res_y = 30.0 / (N_SCAN - 1); // Vertical resolution
            float verticalAngle = atan2(new_point.z, sqrt(new_point.x * new_point.x + new_point.y * new_point.y)) * 180 / M_PI;
            float rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            
            new_point.ring = static_cast<uint16_t>(rowIdn);
            new_point.time = (point_id / static_cast<float>(N_SCAN)) * 0.1 / Horizon_SCAN;

            pc_new->points.push_back(new_point);
        }
        publishPoints(pc_new, *pc_msg);
    }
};

/**
 * @brief Main entry point for the node.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarRingConverter>();
    RCLCPP_INFO(node->get_logger(), "Listening to lidar topic ......");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
