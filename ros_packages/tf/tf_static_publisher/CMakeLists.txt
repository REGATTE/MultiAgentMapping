cmake_minimum_required(VERSION 3.8)
project(tf_static_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(static_tf_publisher src/static_tf_publisher.cpp)
target_include_directories(static_tf_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(static_tf_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  static_tf_publisher
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "tf2_ros"
  "nlohmann_json"
)

install(TARGETS static_tf_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
