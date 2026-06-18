cmake_minimum_required(VERSION 3.8)
project(norlab_icp_mapper_ros)

# ── Build type ─────────────────────────────────────────────────────────────────
# Do not hardcode Release here. Let the user or colcon control the build type.
# Default to RelWithDebInfo (optimized + debug symbols) when nothing is set.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
    "Build type: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
  message(STATUS "[norlab_icp_mapper_ros] CMAKE_BUILD_TYPE defaulting to ${CMAKE_BUILD_TYPE}")
endif()

# ── ROS 2 dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(libpointmatcher_ros REQUIRED)

# ── External libraries ─────────────────────────────────────────────────────────
# libpointmatcher: single find_package, no version pinning here
# (version is enforced by norlab_icp_mapper which depends on it).
find_package(libpointmatcher CONFIG REQUIRED)
find_package(norlab_icp_mapper CONFIG REQUIRED)
find_package(OpenMP REQUIRED)

# ── Service interface generation ───────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectory.srv"
  "srv/SaveMap.srv"
  "srv/LoadMap.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(mapper_node
  src/mapper_node.cpp
  src/NodeParameters.cpp
  src/Deskewer.cpp
)

add_executable(offline_icp_engine
  src/offline_icp_engine.cpp
)

# C++17 required for std::optional, structured bindings, and ROS 2 Jazzy.
target_compile_features(mapper_node PUBLIC cxx_std_17)
target_compile_features(offline_icp_engine PUBLIC cxx_std_17)

# Warn on real issues; do not suppress.
target_compile_options(mapper_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Wno-unused-parameter   # ROS callbacks have unused req/res; suppress explicitly
)

target_compile_options(offline_icp_engine PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Wno-unused-parameter
)

target_include_directories(mapper_node PRIVATE
  ${norlab_icp_mapper_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
)

target_include_directories(offline_icp_engine PRIVATE
  ${norlab_icp_mapper_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
)

ament_target_dependencies(mapper_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  std_srvs
  tf2_ros
  tf2
  tf2_geometry_msgs
  libpointmatcher_ros
)

ament_target_dependencies(offline_icp_engine
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  rosbag2_cpp
  rosbag2_storage
  libpointmatcher_ros
)

target_compile_options(mapper_node PRIVATE ${OpenMP_CXX_FLAGS})
target_compile_options(offline_icp_engine PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(mapper_node
  ${norlab_icp_mapper_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries(offline_icp_engine
  ${norlab_icp_mapper_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
  OpenMP::OpenMP_CXX
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mapper_node "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS offline_icp_engine
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/docs")
  install(DIRECTORY docs
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # Lint checks (copyright, uncrustify, etc.) — disable problematic ones for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Unit test: RegistrationQualityGate (no ROS dependency)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_quality_gate.cpp")
    ament_add_gtest(test_quality_gate
      test/test_quality_gate.cpp
    )
    target_compile_features(test_quality_gate PUBLIC cxx_std_17)
  endif()
endif()

ament_package()
