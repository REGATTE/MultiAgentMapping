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
    // Initialization if needed
}

void subGraphMapping::performDistributedMapping(
    const Pose3& pose_to,
    const pcl::PointCloud<PointPose3D>::Ptr& frame_to,
    const rclcpp::Time& timestamp) {
        RCLCPP_INFO(this->get_logger(), "Performing distributed mapping");
    }