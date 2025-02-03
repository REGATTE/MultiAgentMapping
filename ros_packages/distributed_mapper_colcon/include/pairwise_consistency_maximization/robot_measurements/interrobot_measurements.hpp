#ifndef _DISTRIBUTED_MAPPER_INTERROBOT_MEASUREMENTS_H_
#define _DISTRIBUTED_MAPPER_INTERROBOT_MEASUREMENTS_H_

#include "pairwise_consistency_maximization/robot_measurements/robot_measurements.hpp"
#include "pairwise_consistency_maximization/graph_utils/graph_types.hpp"

namespace robot_measurements {

/**
 * @class InterRobotMeasurements
 * @brief Class that encapsulates interrobot measurements between two robots,
 *        including transforms and robot IDs.
 */
    class InterRobotMeasurements : public RobotMeasurements {
        public:
            /**
             * @brief Constructor that initializes the interrobot measurements.
             * 
             * @param transforms The transforms (relative measurements) between the two robots.
             * @param robot1_id ID of the first robot.
             * @param robot2_id ID of the second robot.
             */
            InterRobotMeasurements(const graph_utils::Transforms& transforms,
                                    const unsigned char& robot1_id,
                                    const unsigned char& robot2_id);

            // ---------------- Accessors ----------------

            /**
             * @brief Gets the ID of the first robot.
             * 
             * @return The ID of the first robot.
             */
            unsigned char getRobot1ID() const;

            /**
             * @brief Gets the ID of the second robot.
             * 
             * @return The ID of the second robot.
             */
            unsigned char getRobot2ID() const;

        private:
            unsigned char robot1_id_; ///< ID of the first robot.
            unsigned char robot2_id_; ///< ID of the second robot.
    };

}  // namespace robot_measurements

#endif  // _DISTRIBUTED_MAPPER_INTERROBOT_MEASUREMENTS_H_
