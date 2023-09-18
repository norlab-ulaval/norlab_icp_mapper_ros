cmake_minimum_required(VERSION 3.8)
project(norlab_icp_mapper_ros)

set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(libpointmatcher_ros REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(norlab_icp_mapper REQUIRED)
find_package(imu_odom REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  ${norlab_icp_mapper_INCLUDE_DIRS}
)

add_executable(mapper_node src/mapper_node.cpp src/NodeParameters.cpp src/Trajectory.cpp)

ament_target_dependencies(mapper_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  tf2_ros
  tf2
  libpointmatcher_ros
  map_msgs
  visualization_msgs
  imu_odom
  tf2_geometry_msgs
)
target_link_libraries(mapper_node
  ${norlab_icp_mapper_LIBRARIES}
)

install(TARGETS mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
