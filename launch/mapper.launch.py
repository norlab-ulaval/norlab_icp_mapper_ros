import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory("norlab_icp_mapper_ros")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation time"
            ),
            DeclareLaunchArgument(
                "points_topic",
                default_value="hesai_lidar/points",
                description="Input point cloud topic (single Hesai). "
                            "For dual-LiDAR merged cloud use /merged_points_filtered.",
            ),
            DeclareLaunchArgument(
                "mapping_config",
                default_value=os.path.join(share, "config", "_mapper.yaml"),
                description="Path to the libpointmatcher ICP config YAML. "
                            "Defaults to _mapper.yaml (MTT-154 standalone profile).",
            ),
            DeclareLaunchArgument(
                "compression_voxel_size",
                default_value="0.10",
                description="Voxel size (m) used to compress the map before registration. "
                            "0.10 = high quality; 0.20 = faster / less RAM.",
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
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "odom_frame": "odom",
                        "robot_frame": "base_footprint",
                        "mapping_config": LaunchConfiguration("mapping_config"),
                        "initial_map_file_name": "",
                        "initial_robot_pose": "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]",
                        "final_map_file_name": "map.vtk",
                        "final_trajectory_file_name": "trajectory.tum",
                        "map_publish_rate": 1.0,
                        "map_tf_publish_rate": 50.0,
                        "max_idle_time": 10.0,
                        "is_mapping": True,
                        "is_online": True,
                        "is_3D": True,
                        "save_map_cells_on_hard_drive": False,
                        "publish_tfs_between_registrations": True,
                        "deskew": False,
                        "compression_voxel_size": LaunchConfiguration("compression_voxel_size"),
                    }
                ],
                remappings=[
                    ("points_in", LaunchConfiguration("points_topic")),
                ],
            ),
        ]
    )
