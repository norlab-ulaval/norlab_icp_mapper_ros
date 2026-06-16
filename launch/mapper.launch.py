import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory("norlab_icp_mapper_ros")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true",
            ),
            DeclareLaunchArgument(
                "points_topic",
                default_value="hesai_lidar/points",
                description="Input cloud. Use /merged_points_filtered for dual-LiDAR.",
            ),
            DeclareLaunchArgument(
                "mapping_config",
                default_value=os.path.join(share, "config", "_mapper.yaml"),
                description="libpointmatcher ICP config YAML.",
            ),
            DeclareLaunchArgument(
                "compression_voxel_size", default_value="0.10",
                description="Map voxel compression (m). 0.10=high quality, 0.20=faster.",
            ),
            # ── IMU odometry (optional, mirrors mtt branch behaviour) ──
            # Set enable_imu_odom:=true when no wheel odom is available (e.g. bag
            # replay without the driver running). Requires the imu_odom package.
            # deskew is automatically enabled when imu_odom is active.
            DeclareLaunchArgument(
                "enable_imu_odom", default_value="false",
                description="Launch imu_odom_node (MTi-100) for dead-reckoning odometry.",
            ),
            DeclareLaunchArgument(
                "imu_topic", default_value="/mti100/data",
                description="IMU topic consumed by imu_odom_node.",
            ),
            DeclareLaunchArgument(
                "imu_frame", default_value="mti100",
                description="IMU sensor frame used by imu_odom_node.",
            ),

            Node(
                package="imu_odom",
                executable="imu_odom_node",
                name="imu_odom",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_imu_odom")),
                parameters=[{
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "odom_frame":   "odom",
                    "robot_frame":  "base_footprint",
                    "imu_frame":    LaunchConfiguration("imu_frame"),
                    "real_time":    False,
                    "use_altitude": False,
                }],
                remappings=[("imu_topic", LaunchConfiguration("imu_topic"))],
            ),

            Node(
                package="norlab_icp_mapper_ros",
                executable="mapper_node",
                name="mapper_node",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--log-level", "debug",
                    "--log-level", "rcl:=INFO",
                    "--log-level", "rmw_fastrtps_cpp:=INFO",
                    "--log-level", "rclcpp:=INFO",
                ],
                parameters=[{
                    "use_sim_time":    LaunchConfiguration("use_sim_time"),
                    "odom_frame":      "odom",
                    "robot_frame":     "base_footprint",
                    "mapping_config":  LaunchConfiguration("mapping_config"),
                    "initial_map_file_name":  "",
                    "initial_robot_pose":
                        "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]",
                    "final_map_file_name":        "map.vtk",
                    "final_trajectory_file_name": "trajectory.tum",
                    "map_publish_rate":     1.0,
                    "map_tf_publish_rate":  50.0,
                    "max_idle_time":        10.0,
                    "is_mapping":           True,
                    "is_online":            True,
                    "is_3D":                True,
                    "save_map_cells_on_hard_drive": False,
                    "publish_tfs_between_registrations": True,
                    # deskew: enable only with trustworthy IMU TFs.
                    # Activate via enable_imu_odom:=true which starts imu_odom_node.
                    "deskew":               False,
                    "compression_voxel_size": LaunchConfiguration("compression_voxel_size"),
                }],
                remappings=[
                    ("points_in", LaunchConfiguration("points_topic")),
                ],
            ),
        ]
    )
