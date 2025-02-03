#ifndef _ROBOT_LOCAL_MAP_H_
#define _ROBOT_LOCAL_MAP_H_

#include "pairwise_consistency_maximization/robot_measurements/robot_measurements.hpp"
#include "pairwise_consistency_maximization/graph_utils/graph_types.hpp"

namespace robot_measurements {

    /**
     * @brief The RobotLocalMap class extends RobotMeasurements to include 
     *        the robot's local trajectory for performing local mapping and consistency checks.
     */
    class RobotLocalMap : public RobotMeasurements {
    public:
        /**
         * @brief Constructor that initializes the robot local map with transforms and loop closures.
         * 
         * @param transforms Map of transformations (typically odometry and loop closures).
         * @param loop_closures List of pairs representing loop closure connections.
         */
        RobotLocalMap(const graph_utils::Transforms& transforms,
                      const graph_utils::LoopClosures& loop_closures);

        /**
         * @brief Constructor that initializes the robot local map with trajectory, transforms, and loop closures.
         * 
         * @param trajectory Precomputed trajectory of the robot.
         * @param transforms Map of transformations (typically odometry and loop closures).
         * @param loop_closures List of pairs representing loop closure connections.
         */
        RobotLocalMap(const graph_utils::Trajectory& trajectory,
                      const graph_utils::Transforms& transforms,
                      const graph_utils::LoopClosures& loop_closures);

        /**
         * @brief Default constructor.
         */
        RobotLocalMap();

        // ------------------ Mutators ------------------

        /**
         * @brief Adds a transformation (edge) to the local map using the given GTSAM BetweenFactor.
         * 
         * @param factor BetweenFactor containing relative pose information between two nodes.
         * @param covariance_matrix Covariance matrix associated with the transformation.
         */
        virtual void addTransform(const gtsam::BetweenFactor<gtsam::Pose3>& factor, 
                                  const gtsam::Matrix& covariance_matrix) override;

        // ------------------ Accessors ------------------

        /**
         * @brief Returns the trajectory associated with this local map.
         * 
         * @return Const reference to the local trajectory of the robot.
         */
        const graph_utils::Trajectory& getTrajectory() const;

    private:
        graph_utils::Trajectory trajectory_; ///< Local trajectory of the robot.
    };

} // namespace robot_measurements

#endif // _ROBOT_LOCAL_MAP_H_
