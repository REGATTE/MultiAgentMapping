#ifndef _SUBGRAPH_PARAMS_H_
#define _SUBGRAPH_PARAMS_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform.hpp>

// pcl
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// mapping
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

using namespace gtsam;
using namespace std;

typedef pcl::PointXYZI PointPose3D;
struct PointPose6D {
    float x; float y; float z; float intensity;
    float roll; float pitch; float yaw;
    double time;
};
POINT_CLOUD_REGISTER_POINT_STRUCT  (PointPose6D,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

class subGraphParams : public rclcpp::Node {
    public:
        subGraphParams();

        gtsam::Pose3 pclPointTogtsamPose3(PointPose6D point);

    protected:
        // ROS Node params
        rclcpp::Node::SharedPtr node_;

        // Robot team information
        int number_of_robots_; // Number of robots in the team

        // Robot info
        std::string name_; // This robot's name
        int id_;           // This robot's ID

        // Keyframe parameters
        float keyframe_distance_threshold_;
        float keyframe_angle_threshold_;
};                                  

#endif // _SUBGRAPH_PARAMS_H_