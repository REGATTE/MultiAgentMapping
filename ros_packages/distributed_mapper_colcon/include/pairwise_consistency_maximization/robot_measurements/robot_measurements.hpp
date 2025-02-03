#ifndef _ROBOT_MEASUREMENTS_H_
#define _ROBOT_MEASUREMENTS_H_

#include "pairwise_consistency_maximization/graph_utils/graph_types.hpp"
#include <gtsam/slam/BetweenFactor.h>

namespace robot_measurements {

    // Class for single robot measurements.
    // The measurements are stored in a Transform map, and LoopClosures contain the node IDs of the loop closures.
    class RobotMeasurements {
    public:
        /**
         * Constructor that initializes RobotMeasurements with given transforms and loop closures.
         */
        RobotMeasurements(const graph_utils::Transforms& transforms,
                          const graph_utils::LoopClosures& loop_closures);

        /**
         * Default constructor.
         */
        RobotMeasurements();

        // ------------------ Mutators ------------------

        /**
         * Adds a transform in the map using a GTSAM BetweenFactor and associated covariance matrix.
         *
         * @param factor BetweenFactor containing the relative pose information.
         * @param covariance_matrix Covariance associated with the transform.
         */
        virtual void addTransform(const gtsam::BetweenFactor<gtsam::Pose3>& factor, 
                                  const gtsam::Matrix& covariance_matrix);
        
        /**
         * Removes a transform from the map based on the given key pair (indices).
         *
         * @param index Pair of keys identifying the transform to remove.
         */
        void removeTransform(const std::pair<gtsam::Key, gtsam::Key>& index);

        // ------------------ Accessors ------------------

        /**
         * Retrieves the map of transforms.
         *
         * @return Reference to the stored transforms.
         */
        virtual const graph_utils::Transforms& getTransforms() const;

        /**
         * Gets the number of poses currently stored in the robot's local map.
         *
         * @return Number of poses.
         */
        virtual const size_t& getNumPoses() const;

        /**
         * Retrieves the list of loop closures.
         *
         * @return Vector of loop closure pairs.
         */
        virtual const graph_utils::LoopClosures& getLoopClosures() const;

        /**
         * Gets the number of degrees of freedom of the robot's local map (typically 6 in 3D, 3 in 2D).
         *
         * @return Number of degrees of freedom.
         */
        virtual const uint8_t& getNbDegreeFreedom() const;

    protected:
        graph_utils::Transforms transforms_;  ///< Map containing all the local measurements of the robot.
        size_t num_poses_;                    ///< Number of poses in the map.
        graph_utils::LoopClosures loop_closures_;  ///< Vector containing the ID pairs of the loop closures.
        uint8_t nb_degree_freedom_;           ///< Number of degrees of freedom, typically 3 in 2D or 6 in 3D.
        bool id_initialized_;                 ///< Flag to indicate whether start_id and end_id of the transforms are initialized.
    };

} // namespace robot_measurements

#endif // _ROBOT_MEASUREMENTS_H_
