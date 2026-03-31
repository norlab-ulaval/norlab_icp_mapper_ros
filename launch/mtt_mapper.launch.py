import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()
    share_folder = get_package_share_directory("norlab_icp_mapper_ros")
    launch_folder = os.path.join(share_folder, "launch")

    # Bag path argument
    ld.add_action(
        DeclareLaunchArgument(
            "bag_path", description="Path to the rosbag file to play offline"
        )
    )

    output_path = LaunchConfiguration('output_path')

    # 2. Standard Rosbag Play Node
    rosbag_play_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play',
                LaunchConfiguration('bag_path'),
                '--rate', '1.0',
                '--clock',
                '--storage', 'mcap',
                # Add your exclusions here if needed:
                '--exclude-topics', '/mapping/icp_odom', '/mapping/map', '/tf'
            ],
            output='screen'
        )

    ld.add_action(rosbag_play_cmd)

    play_rate_control_node = Node(
            package="norlab_icp_mapper_ros",
            executable="slow_down_playbag.py", # Notice we use the filename
            name="bag_speed_manager",
            parameters=[{"use_sim_time": True}]
        )
    ld.add_action(play_rate_control_node)

    odom_config_file = os.path.join(share_folder, "config", "_imu_odom.yaml")
    imu_odom_node = Node(
        package="imu_odom",
        executable="imu_odom_node",
        name="imu_odom",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "odom_frame": "odom",
                "robot_frame": "base_footprint",
                "imu_frame": "mti100",
                "real_time": False,
                "use_altitude": False,
            },
        ],
        remappings=[
            ("imu_topic", "/mti100/data")
        ]
    )
    ld.add_action(imu_odom_node)

    mapping_node = Node(
        package="norlab_icp_mapper_ros",
        executable="mapper_node",
        name="mapper_node",
        output="screen",
        sigterm_timeout="60",  # Wait 60 seconds before escalating to SIGTERM
        sigkill_timeout="10",  # Wait 10 more seconds before SIGKILL
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
                "use_sim_time": True,
                "odom_frame": "odom",
                "robot_frame": "base_link",
                "mapping_config": os.path.join(
                    get_package_share_directory("norlab_icp_mapper_ros"),
                    "config",
                    f"_mapper.yaml",
                ),
                "initial_map_file_name": "",
                "initial_robot_pose": "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]",
                "final_map_file_name": PathJoinSubstitution([
                    LaunchConfiguration("bag_path"),
                    "map.vtk"
                ]),
                "final_trajectory_file_name": PathJoinSubstitution([
                    LaunchConfiguration("bag_path"),
                    "trajectory.tum"
                ]),
                "map_publish_rate": 10.0,
                "map_tf_publish_rate": 10.0,
                "max_idle_time": 5.0,
                "is_mapping": True,
                "is_online": False,
                "is_3D": True,
                "save_map_cells_on_hard_drive": False,
                "publish_tfs_between_registrations": True,
                "deskew": False,
                "compression_voxel_size": 0.5,
            }
        ],
        remappings=[
            ("scan_after_input_filters", f"points_after_input_filters"),
            ("scan_after_deskew", f"points_after_deskew"),
            ("icp_odom", "estimated_odom"),
            ("/tf", "tf"),
        ],
    )

    ld.add_action(mapping_node)

    # Auto-shutdown the launch run when the mapper finishes processing offline scans
    shutdown_action = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mapping_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )
    ld.add_action(shutdown_action)

    return ld
