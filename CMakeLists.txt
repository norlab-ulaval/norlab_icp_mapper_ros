cmake_minimum_required(VERSION 3.8)
project(norlab_icp_mapper_ros)

set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(libpointmatcher_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(norlab_icp_mapper REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SaveTrajectory.srv"
    "srv/SaveMap.srv"
    "srv/LoadMap.srv"
    DEPENDENCIES std_msgs geometry_msgs
)

include_directories(
    ${norlab_icp_mapper_INCLUDE_DIRS}
)

add_executable(mapper_node src/mapper_node.cpp src/NodeParameters.cpp)

ament_target_dependencies(mapper_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    std_srvs
    tf2_ros
    tf2
    libpointmatcher_ros
)
target_link_libraries(mapper_node
    ${norlab_icp_mapper_LIBRARIES}
)

rosidl_target_interfaces(mapper_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

install(TARGETS mapper_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

