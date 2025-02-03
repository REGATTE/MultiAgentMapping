#ifndef _GRAPH_UTILS_TYPES_H_
#define _GRAPH_UTILS_TYPES_H_

#include <boost/shared_ptr.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>

#include <map>
#include <vector>
#include <Eigen/Dense>

namespace graph_utils {
    struct PoseWithCovariance {
        gtsam::Pose3 pose;
        gtsam::Matrix covariance_matrix;
    };

    // Struct to define transformations between 2 points
    struct Transform {
        gtsam::Key i, j;
        graph_utils::PoseWithCovariance pose;
        bool is_loopclosure;
    };

    // struct to define a std::map of transformations
    struct Transforms {
        gtsam::Key start_id, end_id;
        std::map<std::pair<gtsam::Key, gtsam::Key>, graph_utils::Transform> transforms;
    };

    // Struct defining a pose in the robot trajectory
    struct TrajectoryPose {
        gtsam::Key id;
        graph_utils::PoseWithCovariance pose;
    };

    // Struct defining a robot trajectory
    struct Trajectory {
        gtsam::Key start_id, end_id;
        std::map<gtsam::Key, graph_utils::TrajectoryPose> trajectory_poses;
    };

    // type of store poses ID's of loop closures - list of pair of poses
    typedef std::vector<std::pair<gtsam::Key, gtsam::Key>> LoopClosures;

    // type to store pose error vector and its associated covariance - pair vector-matrix
    typedef std::pair<gtsam::Vector6, gtsam::Matrix> ConsistencyErrorData;

    // Covariance matrix with usual value (rotation std: 0.01 rad, translation std: 0.1 m).
    const gtsam::Matrix FIXED_COVARIANCE = 
        (Eigen::MatrixXd(6, 6) << 0.0001, 0, 0, 0, 0, 0,
                                  0, 0.0001, 0, 0, 0, 0,
                                  0, 0, 0.0001, 0, 0, 0,
                                  0, 0, 0, 0.01, 0, 0,
                                  0, 0, 0, 0, 0.01, 0,
                                  0, 0, 0, 0, 0, 0.01).finished();
};

#endif // _GRAPH_UTILS_TYPES_H_
