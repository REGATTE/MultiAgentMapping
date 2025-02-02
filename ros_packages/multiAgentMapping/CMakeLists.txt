cmake_minimum_required(VERSION 3.5)
project(multi_agent_mapping)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)

# Common dependencies
set(COMMON_DEPENDENCIES 
  rclcpp 
  rclpy 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  nav_msgs 
  pcl_conversions 
  pcl_msgs 
  visualization_msgs 
  tf2 
  tf2_ros 
  tf2_eigen 
  tf2_sensor_msgs 
  tf2_geometry_msgs 
  OpenCV 
  PCL
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" "srv/SaveMap.srv" DEPENDENCIES std_msgs sensor_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# build library from src/Distributed_Mapping
add_library(distributed_mapping_lib
  src/Distributed_Mapping/subGraphsUtils.cpp
)
target_include_directories(distributed_mapping_lib PUBLIC ${rclcpp_INCLUDE_DIRS})
ament_target_dependencies(distributed_mapping_lib ${COMMON_DEPENDENCIES} GTSAM Eigen3)
target_link_libraries(distributed_mapping_lib  ${PCL_LIBRARIES}  ${OpenCV_LIBRARIES}  ${Boost_LIBRARIES}  ${GTSAM_LIBRARIES}  OpenMP::OpenMP_CXX)

add_executable(${PROJECT_NAME}_featureExtraction src/LIO_SAM/featureExtraction.cpp)
ament_target_dependencies(${PROJECT_NAME}_featureExtraction rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
target_link_libraries(${PROJECT_NAME}_featureExtraction "${cpp_typesupport_target}") 

add_executable(${PROJECT_NAME}_imageProjection src/LIO_SAM/imageProjection.cpp)
ament_target_dependencies(${PROJECT_NAME}_imageProjection rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
target_link_libraries(${PROJECT_NAME}_imageProjection "${cpp_typesupport_target}") 

add_executable(${PROJECT_NAME}_imuPreintegration src/LIO_SAM/imuPreintegration.cpp)
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM Eigen)
target_link_libraries(${PROJECT_NAME}_imuPreintegration gtsam "${cpp_typesupport_target}")

add_executable(${PROJECT_NAME}_mapOptimization src/LIO_SAM/mapOptimization.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapOptimization rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM)
if (OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam "${cpp_typesupport_target}" OpenMP::OpenMP_CXX distributed_mapping_lib)
else()
  target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam "${cpp_typesupport_target}" distributed_mapping_lib)
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Executables
install(
  TARGETS 
    ${PROJECT_NAME}_featureExtraction
    ${PROJECT_NAME}_imageProjection
    ${PROJECT_NAME}_imuPreintegration
    ${PROJECT_NAME}_mapOptimization
  DESTINATION lib/${PROJECT_NAME}
)

# Install Library
install(
  TARGETS distributed_mapping_lib
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

# Export Include Directories
ament_export_include_directories(include)

ament_package()
