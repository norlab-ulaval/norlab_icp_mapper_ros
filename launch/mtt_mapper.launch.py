import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mapping_config = LaunchConfiguration("mapping_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation time"
            ),
            DeclareLaunchArgument(
                "odom_frame", default_value="odom", description="Odom frame (e.g. odom or base_footprint)"
            ),
            DeclareLaunchArgument(
                "mapping_config",
                default_value=os.path.join(
                    get_package_share_directory("norlab_icp_mapper_ros"),
                    "config",
                    "mtt",
                    "mapper_hesai_garage.yaml",
                ),
                description="Path to the mapper YAML config",
            ),
            Node(
                package="norlab_icp_mapper_ros",
                executable="mapper_node",
                name="mapper_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "robot_frame": "base_footprint",
                        "mapping_config": mapping_config,
                        "initial_map_file_name": "",
                        "initial_robot_pose": "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]",
                        "final_map_file_name": "map_hesai.vtk",
                        "final_trajectory_file_name": "trajectory_hesai.vtk",
                        "map_publish_rate": 5.0,
                        "map_tf_publish_rate": 5.0,
                        "max_idle_time": 5.0,
                        "is_mapping": True,
                        "is_online": True,
                        "is_3D": True,
                        "save_map_cells_on_hard_drive": False,
                        "publish_tfs_between_registrations": True,
                        "deskew": False,
                        "compression_voxel_size": 0.3,
                    }
                ],
                remappings=[
                    ("points_in", "/hesai_lidar/points"),
                    ("map", "/mapping/map"),
                    ("icp_odom", "/mapping/icp_odom"),
                    ("scan_after_input_filters", "/mapping/scan_after_input_filters"),
                    ("scan_after_deskew", "/mapping/scan_after_deskew"),
                ],
            ),
        ]
    )
