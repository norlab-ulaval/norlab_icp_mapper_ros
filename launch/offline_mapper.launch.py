import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

IS_MAPPING = os.getenv("IS_MAPPING")
STORAGE_PATH = os.getenv("STORAGE_PATH")
INPUT_IMU_BIAS_FILE = os.path.join("/", "calib", "imu.json")
LIDAR_TYPE = "robosense"
NAMESPACE = os.getenv("NAMESPACE")

if IS_MAPPING is None:
    print("IS_MAPPING is not set")
    exit(1)
elif STORAGE_PATH is None:
    print("STORAGE_PATH is not set")
    exit(1)

IS_MAPPING = IS_MAPPING == "1"
if IS_MAPPING:
    input_map_name = ""
    output_map_name = f"{STORAGE_PATH}/map.vtk"
else:
    input_map_name = f"{STORAGE_PATH}/map.vtk"
    output_map_name = ""


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

    # Note: we are enforcing use_sim_time=true for offline playback
    # Instead of exposing use_sim_time, we hardcode it to true for the nodes.

    offline_player_node = Node(
        package="norlab_icp_mapper_ros",
        executable="offline_player_node",
        name="offline_player_node",
        namespace=NAMESPACE,
        output="screen",
                "odom_topic": ("" if not NAMESPACE else f"/{NAMESPACE}")
                + "/estimated_odom",
                "max_buffer_size": 2,
            }
        ],
    )
    ld.add_action(offline_player_node)

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_folder, "imu.launch.py")])
    )
    ld.add_action(imu_launch)

    ekf_config_file = os.path.join(share_folder, "config", "_ekf.yaml")

    ekf_odom_node = Node(
        package="robot_localization",
        executable="ekf_node",
        # namespace=NAMESPACE,
        name="ekf_odom_node",
        output="screen",
        respawn=True,
        parameters=[
            ekf_config_file,
            {
                "use_sim_time": True,
            },
        ],
        arguments=["--ros-args", "--log-level", "ERROR"],
        remappings=[
            ("odometry/filtered", "ekf/odom"),
            ("/tf", ("" if not NAMESPACE else f"/{NAMESPACE}") + "/tf"),
        ],
    )

    mapping_node = Node(
        package="norlab_icp_mapper_ros",
        executable="mapper_node",
        name="mapper_node",
        namespace=NAMESPACE,
        output="screen",
        sigterm_timeout="600",  # Wait 60 seconds before escalating to SIGTERM
        sigkill_timeout="100",  # Wait 10 more seconds before SIGKILL
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
                    f"_mapper_{LIDAR_TYPE}.yaml",
                ),
                "initial_map_file_name": input_map_name,
                "initial_robot_pose": "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]",
                "final_map_file_name": output_map_name,
                "final_trajectory_file_name": output_map_name.replace(
                    "map.vtk", "trajectory.tum"
                ),
                "map_publish_rate": 10.0,
                "map_tf_publish_rate": 10.0,
                "max_idle_time": 100.0,
                "is_mapping": IS_MAPPING,
                "is_online": False,
                "is_3D": True,
                "save_map_cells_on_hard_drive": False,
                "publish_tfs_between_registrations": True,
                "deskew": True,
                "compression_voxel_size": 0.5,
            }
        ],
        remappings=[
            ("points_in", f"/{LIDAR_TYPE}/points"),
            ("scan_after_input_filters", f"/{LIDAR_TYPE}/points_after_input_filters"),
            ("scan_after_deskew", f"/{LIDAR_TYPE}/points_after_deskew"),
            ("icp_odom", "estimated_odom"),
            ("/tf", "tf"),
        ],
    )

    ld.add_action(ekf_odom_node)
    ld.add_action(mapping_node)

    # Auto-shutdown the launch run when the mapper finishes processing offline scans
    shutdown_action = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mapping_node, on_exit=[EmitEvent(event=Shutdown())]
    shutdown_action = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mapping_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )
    ld.add_action(shutdown_action)

    return ld
