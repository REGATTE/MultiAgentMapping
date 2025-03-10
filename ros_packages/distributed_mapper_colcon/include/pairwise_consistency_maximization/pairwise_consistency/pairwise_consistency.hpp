#ifndef _PAIRWISE_CONSISTENCY_H_
#define _PAIRWISE_CONSISTENCY_H_

#include "pairwise_consistency_maximization/graph_utils/graph_utils_functions.hpp"
#include <eigen3/Eigen/Geometry>

namespace pairwise_consistency {
    class PairwiseConsistency{
        public:
            PairwiseConsistency(const graph_utils::Transforms& transforms_robot1,
                            const graph_utils::Transforms& transforms_robot2,
                            const graph_utils::Transforms& transforms_interrobot,
                            const graph_utils::LoopClosures& loop_closures,
                            const graph_utils::Trajectory& trajectory_robot1,
                            const graph_utils::Trajectory& trajectory_robot2,
                            const uint8_t& nb_degree_freedom,
                            const double& pcm_threshold):
                                loop_closures_(loop_closures), 
                                transforms_robot1_(transforms_robot1), 
                                transforms_robot2_(transforms_robot2), 
                                transforms_interrobot_(transforms_interrobot),
                                trajectory_robot1_(trajectory_robot1), 
                                trajectory_robot2_(trajectory_robot2),
                                nb_degree_freedom_(nb_degree_freedom), 
                                pcm_threshold_(pcm_threshold){};
            
            // computation of the consistency matrix
            Eigen::MatrixXi computeConsistentMeasurementsMatrix();

            // Accessors
            // return list of loop closures
            const graph_utils::LoopClosures& getLoopClosures() const;

            // returns map of the transforms of robot 1
            const graph_utils::Transforms& getTransformsRobot1() const;

            // returns map of the transforms of robot 2
            const graph_utils::Transforms& getTransformsRobot2() const;

            // returns map of interrobot transforms
            const graph_utils::Transforms& getTransformsInterRobot() const;
        
        private:
            /**
             * \brief Computes the consistency loop : aXij + abZjl + bXlk - abZik (see references)
             *
             * @param aXij Transformation between poses i and j on robot A trajectory
             * @param bXlk Transformation between poses l and k on robot B trajectory
             * @param abZik Transformation between pose i of robot A trajectory and pose k of robot B trajectory
             * @param abZjl Transformation between pose j of robot A trajectory and pose l of robot B trajectory
             * @returns Consistency error data : pose error vector and its associated covariance
             */
            graph_utils::ConsistencyErrorData computeConsistencyError(const graph_utils::PoseWithCovariance& aXij,
                                                                      const graph_utils::PoseWithCovariance& bXlk,
                                                                      const graph_utils::PoseWithCovariance& abZik,
                                                                      const graph_utils::PoseWithCovariance& abZjl);
            
            /**
             * \brief Compute the Mahalanobis Distance of the input pose (result of pose_a-pose_b)
             *
             * @param transform pose measurement describing the difference between two poses.
             * @returns Mahalanobis Distance
             */
            double computeSquaredMahalanobisDistance(const graph_utils::ConsistencyErrorData& consistency_error);

            /**
             * \brief This function returns the pose with covariance obtained
             * from the composition of the two specified poses on the desired robot trajectory
             *
             * @param id1 first pose ID
             * @param id2 second pose ID
             * @param robot_id robot ID to select to desired trajectory
             * @return pose with covariance between the two poses
             */
            graph_utils::PoseWithCovariance composeOnTrajectory(const size_t& id1, const size_t& id2, const size_t& robot_id);

            /**
             * \brief This function returns the chi-squared threshold given the confidence probability
             *
             * @return the threshold to be used on the error to determine if two measurements are consistent
             */
            double getChiSquaredThreshold();

            graph_utils::LoopClosures loop_closures_;///< loop_closures to consider

            graph_utils::Transforms transforms_robot1_, transforms_robot2_, transforms_interrobot_;///< Measurements for each robot

            graph_utils::Trajectory trajectory_robot1_, trajectory_robot2_;///< Trajectory of the robots

            uint8_t nb_degree_freedom_;///< Number of degree of freedom of the measurements.

            double pcm_threshold_;///< Probability of a larger value of X^2.
    };
}

#endif // _PAIRWISE_CONSISTENCY_H_