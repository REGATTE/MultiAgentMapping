cmake_minimum_required(VERSION 3.5)
project(distributed_mapper)

# Set C++ standard and build type
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  src/pairwise_consistency_maximization
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/
)

# ===================== Graph Utils Library =====================
add_library(graph_utils
  src/pairwise_consistency_maximization/graph_utils/graph_utils_functions.cpp
)
target_link_libraries(graph_utils gtsam Eigen3::Eigen)

# ===================== Robot Measurements Library =====================
add_library(robot_measurements
  src/pairwise_consistency_maximization/robot_measurements/robot_measurements.cpp
  src/pairwise_consistency_maximization/robot_measurements/robot_local_map.cpp
  src/pairwise_consistency_maximization/robot_measurements/interrobot_measurements.cpp
)
target_link_libraries(robot_measurements graph_utils gtsam Eigen3::Eigen)

# ===================== Pairwise Consistency Compute Library =====================
add_library(pairwise_consistency
  src/pairwise_consistency_maximization/pairwise_consistency/pairwise_consistency.cpp
)
target_link_libraries(pairwise_consistency robot_measurements gtsam Eigen3::Eigen)

# ===================== Max Clique Solver Library =====================
add_library(fast_max-clique_finder
  src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/findClique.hpp
  src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/graphIO.hpp 
  src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/findClique.cpp 
  src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/findCliqueHeu.cpp  
  src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/utils.cpp  
  src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/graphIO.cpp
)
target_compile_options(fast_max-clique_finder PRIVATE -w)

# ===================== GlobalMap Library =====================
add_library(global_map_solver 
  src/pairwise_consistency_maximization/global_map/global_map.cpp
)
target_link_libraries(global_map_solver graph_utils pairwise_consistency fast_max-clique_finder)

# ===================== Distributed PCM Library =====================
add_library(distributed_pcm
  src/pairwise_consistency_maximization/distributed_pcm/distributed_pcm.cpp
)
target_link_libraries(distributed_pcm global_map_solver)

# ===================== Distributed Mapper Library =====================
add_library(distributed_mapper
  src/distributed_mapper/distributed_mapper.cpp 
  src/distributed_mapper/evaluation_utils.cpp 
  include/distributed_mapper/distributed_mapper_utils.hpp 
  src/distributed_mapper/distributed_mapper_utils.cpp 
  src/distributed_mapper/run_distributed_mapper.cpp
)
target_link_libraries(distributed_mapper graph_utils robot_measurements pairwise_consistency fast_max-clique_finder distributed_pcm global_map_solver)


# ===================== Installation =====================
install(
  TARGETS graph_utils robot_measurements pairwise_consistency fast_max-clique_finder global_map_solver distributed_pcm distributed_mapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include/
)

# Export Include Directories
ament_export_include_directories(include)
ament_package()
