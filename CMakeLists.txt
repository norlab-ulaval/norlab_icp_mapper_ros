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
find_package(rosidl_default_generators REQUIRED)
find_package(libpointmatcher CONFIG 1.4.2 REQUIRED)
find_package(norlab_icp_mapper CONFIG 2.1.1 REQUIRED)
find_package(libpointmatcher_ros 2.0.1 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(libpointmatcher CONFIG 1.4.3 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SaveTrajectory.srv"
    "srv/SaveMap.srv"
    "srv/LoadMap.srv"
    DEPENDENCIES std_msgs geometry_msgs
)

include_directories(
    ${norlab_icp_mapper_INCLUDE_DIRS}
    ${libpointmatcher_INCLUDE_DIRS}
)

add_executable(mapper_node
    src/mapper_node.cpp
    src/NodeParameters.cpp
    src/Deskewer.cpp
)

ament_target_dependencies(mapper_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    std_srvs
    tf2_ros
    tf2
    libpointmatcher_ros
    tf2_geometry_msgs
)

# Add OpenMP flags and link OpenMP library
target_compile_options(mapper_node PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(mapper_node
    ${norlab_icp_mapper_LIBRARIES}
    ${libpointmatcher_LIBRARIES}
    OpenMP::OpenMP_CXX
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mapper_node "${cpp_typesupport_target}")
ament_export_dependencies(rosidl_default_runtime)

add_executable(offline_player_node
    src/offline_player_node.cpp
)

ament_target_dependencies(offline_player_node
    rclcpp
    rosbag2_cpp
    rosbag2_storage
    rosgraph_msgs
    nav_msgs
)

install(TARGETS mapper_node offline_player_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/slow_down_playbag.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
