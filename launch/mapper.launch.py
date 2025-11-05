import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            Node(
                package="norlab_icp_mapper_ros",
                executable="mapper_node",
                namespace="mapping",
                name="mapper_node",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    "debug",
                    "--log-level",
                    "rcl:=INFO",
                    "--log-level",
                    "rmw_fastrtps_cpp:=INFO",
                    "--log-level",
                    "rclcpp:=INFO",
                ],
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "odom_frame": "odom",
                        "robot_frame": "base_link",
                        "mapping_config": os.path.join(
                            get_package_share_directory("norlab_icp_mapper_ros"),
                            "config",
                            "mapper.yaml",
                        ),
                        "initial_map_file_name": "",
                        "initial_robot_pose": "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]",
                        "final_map_file_name": "map.vtk",
                        "final_trajectory_file_name": "trajectory.vtk",
                        "map_publish_rate": 10.0,
                        "map_tf_publish_rate": 10.0,
                        "max_idle_time": 10.0,
                        "is_mapping": True,
                        "is_online": True,
                        "is_3D": True,
                        "save_map_cells_on_hard_drive": True,
                        "publish_tfs_between_registrations": True,
                        "deskew": False,
                        "compression_voxel_size": 0.2,
                    }
                ],
                remappings=[
                    ("points_in", "/rslidar128/points"),
                ],
            ),
        ]
    )
