#include "pairwise_consistency_maximization/global_map/global_map.hpp"
#include <cmath>
#include <gtsam/inference/Symbol.h>
#include "findClique.hpp"
#include <filesystem>
#include <fstream>

#define DIR std::string(std::getenv("HOME")) + "/distributed_mapping_output"

namespace fs = std::filesystem;  // Add namespace alias for convenience

namespace global_map {
    const std::string GlobalMap::LOG_DIRECTORY = DIR + "/";
    const std::string GlobalMap::CONSISTENCY_MATRIX_FILE_NAME = "consistency_matrix";
    const std::string GlobalMap::CONSISTENCY_LOOP_CLOSURES_FILE_NAME = "consistent_loop_closures";

    GlobalMap::GlobalMap(const robot_measurements::RobotLocalMap& robot1_local_map,
                            const robot_measurements::RobotLocalMap& robot2_local_map,
                            const robot_measurements::RobotMeasurements& interrobot_measurements,
                            const double& pcm_threshold,
                            const bool& use_heuristics):
                                pairwise_consistency_(robot1_local_map.getTransforms(), robot2_local_map.getTransforms(), 
                                                        interrobot_measurements.getTransforms(), interrobot_measurements.getLoopClosures(),
                                                        robot1_local_map.getTrajectory(), robot2_local_map.getTrajectory(),
                                                        robot1_local_map.getNbDegreeFreedom(), pcm_threshold), use_heuristics_(use_heuristics) {}
    
    std::pair<std::vector<int>, int> GlobalMap::pairwiseConsistencyMaximization() {
        // compute consistency Matrix
        std::cout << "[GlobalMap] Computing consistency matrix..." << std::endl;
        Eigen::MatrixXi consistency_matrix = pairwise_consistency_.computeConsistentMeasurementsMatrix();
        std::cout << "[GlobalMap] Consistency matrix size: " << consistency_matrix.rows() 
                << "x" << consistency_matrix.cols() << std::endl;

        // Create directory if it doesn't exist
        fs::path dir_path(LOG_DIRECTORY);
        if (!fs::exists(dir_path)) {
            std::cout << "[GlobalMap] Creating directory: " << LOG_DIRECTORY << std::endl;
            fs::create_directories(dir_path);
        }

        char robot_id = gtsam::Symbol(pairwise_consistency_.getTransformsRobot1().start_id).chr();
        // Use LOG_DIRECTORY in the file path
        std::string consistency_matrix_file = LOG_DIRECTORY + CONSISTENCY_MATRIX_FILE_NAME + "_" + robot_id + ".clq.mtx";
        
        std::cout << "[GlobalMap] Writing consistency matrix to file: " 
                << consistency_matrix_file << std::endl;
        graph_utils::printConsistencyGraph(consistency_matrix, consistency_matrix_file);

        // Compute maximum clique
        FMC::CGraphIO gio;
        try {
            std::cout << "[GlobalMap] Reading graph from file..." << std::endl;
            gio.readGraph(consistency_matrix_file);
            
            std::cout << "[GlobalMap] Graph size: " << consistency_matrix.rows() << " vertices" << std::endl;
            
            int max_clique_size = 0;
            std::vector<int> max_clique_data;
            
            if (consistency_matrix.rows() <= 0) {
                std::cerr << "[GlobalMap] Invalid matrix size: " << consistency_matrix.rows() << std::endl;
                throw std::runtime_error("Invalid graph size");
            }
            
            max_clique_data.reserve(consistency_matrix.rows());  // Reserve with known size
            
            std::cout << "[GlobalMap] Computing maximum clique..." << std::endl;
            max_clique_size = FMC::maxCliqueHeu(gio, max_clique_data);
            
            std::cout << "[GlobalMap] Max clique size: " << max_clique_size << std::endl;
            
            return std::make_pair(max_clique_data, 
                pairwise_consistency_.getLoopClosures().size() - max_clique_data.size());
        } catch (const std::exception& e) {
            std::cerr << "[GlobalMap] Failed in max clique computation: " << e.what() << std::endl;
            throw;
        }
    }
}