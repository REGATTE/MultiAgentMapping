#include "multiAgentMapping/distributedMapping/subGraphParams.hpp"

subGraphParams::subGraphParams() : rclcpp::Node("subgraph_params"){

}

gtsam::Pose3 subGraphParams::pclPointTogtsamPose3(PointPose6D point)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(point.roll), double(point.pitch), double(point.yaw)),
                        gtsam::Point3(double(point.x), double(point.y), double(point.z)));
}