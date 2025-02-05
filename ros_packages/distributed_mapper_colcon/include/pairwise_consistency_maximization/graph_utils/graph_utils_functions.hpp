#ifndef _GRAPH_UTILS_FUNCTIONS_H_
#define _GRAPH_UTILS_FUNCTIONS_H_

#include "pairwise_consistency_maximization/graph_utils/graph_types.hpp"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

namespace graph_utils {

    /**
     * @brief Parses a .g2o file and extracts poses, transforms, and loop closures.
     * 
     * @param file_name The path to the .g2o file.
     * @param num_poses The number of poses extracted from the file.
     * @param transforms A container to store relative transforms between poses.
     * @param loop_closures A container to store loop closures between poses.
     * @param only_loop_closures If true, only loop closures are extracted.
     * @return uint8_t Returns 0 on success, or an error code on failure.
     */
    uint8_t parseG2ofile(const std::string &file_name, size_t &num_poses, Transforms& transforms, LoopClosures& loop_closures, const bool& only_loop_closures);

    /**
     * @brief Composes two poses with covariance.
     * 
     * @param a The first pose with covariance.
     * @param b The second pose with covariance.
     * @param out The resulting composed pose (a + b).
     */
    void poseCompose(const graph_utils::PoseWithCovariance &a, const graph_utils::PoseWithCovariance &b, graph_utils::PoseWithCovariance &out);

    /**
     * @brief Computes the inverse of a pose with covariance.
     * 
     * @param a The pose to be inverted.
     * @param out The resulting inverted pose.
     */
    void poseInverse(const graph_utils::PoseWithCovariance &a, graph_utils::PoseWithCovariance &out);

    /**
     * @brief Computes the relative transformation (a - b) between two poses with covariance.
     * 
     * @param a The first pose.
     * @param b The second pose.
     * @param out The resulting relative pose (a - b).
     */
    void poseBetween(const graph_utils::PoseWithCovariance &a, const graph_utils::PoseWithCovariance &b, graph_utils::PoseWithCovariance &out); 

    /**
     * @brief Builds a trajectory by composing successive poses from odometry measurements.
     * 
     * @param transforms The transforms representing the relative pose changes.
     * @return Trajectory The resulting trajectory.
     */
    Trajectory buildTrajectory(const Transforms& transforms);

    /**
     * @brief Prints the consistency matrix to a file in the format required by the maximum clique solver.
     * 
     * @param consistency_matrix The consistency matrix to be printed.
     * @param file_name The output file path.
     */
    void printConsistencyGraph(const Eigen::MatrixXi& consistency_matrix, const std::string& file_name);

    /**
     * @brief Checks if a given pose ID is part of the trajectory.
     * 
     * @param trajectory The trajectory to search within.
     * @param pose_id The pose ID to check.
     * @return true If the pose ID is in the trajectory.
     * @return false If the pose ID is not in the trajectory.
     */
    bool isInTrajectory(const Trajectory& trajectory, const size_t& pose_id);

    /**
     * @brief Prints a list of consistent loop closures to a file.
     * 
     * @param loop_closures The loop closures to be printed.
     * @param max_clique_data The indices of the maximum clique in the consistency graph.
     * @param file_name The output file path.
     */
    void printConsistentLoopClosures(const LoopClosures& loop_closures, const std::vector<int>& max_clique_data, const std::string& file_name);
};

#endif // _GRAPH_UTILS_FUNCTIONS_H_
