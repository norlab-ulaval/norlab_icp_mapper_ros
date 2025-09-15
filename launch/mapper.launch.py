import os
import json
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import (
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler,
    Shutdown,
    LogInfo,
    GroupAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart

# row is map source
# col is rosbag
# BASE_INPUT_PATH = "/home/user/data/lidar-evaluation"
# DEPLOYMENT_ROW_FOLDER = "2024-11-21"
# TRAJECTORY_ROW_FOLDER = f"blue-{DEPLOYMENT_ROW_FOLDER}-10-44"

# DEPLOYMENT_COL_FOLDER = "2025-06-26"
# TRAJECTORY_COL_FOLDER = f"blue-{DEPLOYMENT_COL_FOLDER}-10-35"
# IS_MAPPING = TRAJECTORY_ROW_FOLDER != TRAJECTORY_COL_FOLDER
# if TRAJECTORY_ROW_FOLDER == TRAJECTORY_COL_FOLDER:
#     INPUT_MAP_FILE = ""
# else:
#     INPUT_MAP_FILE = os.path.join(
#         BASE_INPUT_PATH,
#         DEPLOYMENT_ROW_FOLDER,
#         TRAJECTORY_ROW_FOLDER,
#         f"{TRAJECTORY_ROW_FOLDER}_map.vtk",
#     )
# INPUT_IMU_BIAS_FILE = os.path.join(
#     BASE_INPUT_PATH, DEPLOYMENT_COL_FOLDER, TRAJECTORY_COL_FOLDER, "calib", "imu.json"
# )
# OUTPUT_MAP_TRAJ_PATH = os.path.join(
#     BASE_INPUT_PATH, DEPLOYMENT_COL_FOLDER, TRAJECTORY_COL_FOLDER, TRAJECTORY_ROW_FOLDER
# )
# PLAYBACK_RATE = 1.0
# INPUT_BAG = os.path.join(BASE_INPUT_PATH, DEPLOYMENT_COL_FOLDER, TRAJECTORY_COL_FOLDER)
# IMU_TYPE = "vectornav"  # or 'xsens'
# LIDAR_TYPE = "robosense"


def generate_launch_description():
    ld = LaunchDescription()
    share_folder = get_package_share_directory("norlab_icp_mapper_ros")

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation time"
        )
    )

    base_path = os.getenv("BASE_INPUT_PATH", "/home/user/data/lidar-evaluation")
    row_folder = os.getenv("DEPLOYMENT_ROW_FOLDER", "2024-11-21")
    col_folder = os.getenv("DEPLOYMENT_COL_FOLDER", "2025-06-26")
    traj_row = os.getenv("TRAJECTORY_ROW_FOLDER", f"blue-{row_folder}-10-44")
    traj_col = os.getenv("TRAJECTORY_COL_FOLDER", f"blue-{col_folder}-10-35")
    IMU_TYPE = os.getenv("IMU_TYPE", "vectornav")
    LIDAR_TYPE = os.getenv("LIDAR_TYPE", "robosense")
    PLAYBACK_RATE = float(os.getenv("PLAYBACK_RATE", "1.0"))

    IS_MAPPING = traj_row == traj_col
    if IS_MAPPING:
        INPUT_MAP_FILE = ""
    else:
        INPUT_MAP_FILE = os.path.join(
            base_path, row_folder, traj_row, f"{traj_row}_map.vtk"
        )

    INPUT_IMU_BIAS_FILE = os.path.join(
        base_path, col_folder, traj_col, "calib", "imu.json"
    )
    OUTPUT_MAP_TRAJ_PATH = os.path.join(base_path, col_folder, traj_col, traj_row)
    INPUT_BAG = os.path.join(base_path, col_folder, traj_col)

    rosbag_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            INPUT_BAG,
            "--clock",
            "--rate",
            str(PLAYBACK_RATE),
            "--start-offset",
            "0",
            "--start-paused",
            "--read-ahead-queue-size",
            "12000",
            "--log-level",
            "warn",
        ],
        output="log",
        name="rosbag_play",
    )
    save_map_traj_process = ExecuteProcess(
        cmd=[
            "mapper_save_map_traj",
            OUTPUT_MAP_TRAJ_PATH,
        ],
        output="screen",
        name="rosbag_play",
    )

    ld.add_action(GroupAction([LogInfo(msg=f"Playing {INPUT_BAG}"), rosbag_process]))

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=rosbag_process,
                on_exit=[
                    LogInfo(
                        msg="Rosbag playback finished. Saving map and trajectory..."
                    ),
                    save_map_traj_process,
                ],
            )
        )
    )
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=save_map_traj_process,
                on_exit=[
                    LogInfo(msg="Trajectory saved. Shutting down launch..."),
                    Shutdown(),
                ],
            )
        )
    )
    bag_play_process = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/rosbag2_player/resume",
            "rosbag2_interfaces/srv/Resume",
            "{}",
        ],
        output="log",
        name="bag_play_service",
    )
    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=rosbag_process,
                on_start=[
                    TimerAction(
                        period=5.0,
                        actions=[
                            LogInfo(msg="Starting bag play..."),
                            bag_play_process,
                        ],
                    )
                ],
            )
        ),
    )

    if IMU_TYPE == "vectornav":
        namespace = LaunchConfiguration("vn100_ns")
        vectornav_namespace_launch_arg = DeclareLaunchArgument(
            "vn100_ns", default_value=IMU_TYPE
        )

        config_file = os.path.join(share_folder, "config", "_vn100.yaml")

        bias_x = 0.0
        bias_y = 0.0
        bias_z = 0.0

        with open(INPUT_IMU_BIAS_FILE, "r") as f:
            bias_data = json.load(f)
            bias_x = bias_data[IMU_TYPE]["angular_velocity"]["x"]
            bias_y = bias_data[IMU_TYPE]["angular_velocity"]["y"]
            bias_z = bias_data[IMU_TYPE]["angular_velocity"]["z"]

        print(f"Biases: x={bias_x}, y={bias_y}, z={bias_z}")
        bias_compensator_node = Node(
            package="norlab_imu_tools",
            executable="imu_bias_compensator_node",
            name="bias_compensator",
            namespace=namespace,
            output="both",
            parameters=[
                config_file,
                {"bias_x": bias_x, "bias_y": bias_y, "bias_z": bias_z},
            ],
            remappings=[
                ("imu_topic_in", "data_raw"),
                ("bias_topic_in", "bias"),
                ("imu_topic_out", "data_unbiased"),
            ],
            arguments=[
                "--ros-args",
                "--log-level",
                "warn",
            ],
        )

        filter_madgwick_node = Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="madgwick_filter",
            namespace=namespace,
            output="both",
            parameters=[config_file],
            remappings=[
                ("imu/data_raw", "data_unbiased"),
                ("imu/mag", "mag"),
                ("imu/data", "data"),
            ],
            arguments=[
                "--ros-args",
                "--log-level",
                "warn",
            ],
        )
        ld.add_action(vectornav_namespace_launch_arg)
        ld.add_action(
            RegisterEventHandler(
                OnProcessStart(
                    target_action=rosbag_process,
                    on_start=bias_compensator_node,  # Starts when rosbag starts
                )
            ),
        )
        ld.add_action(
            RegisterEventHandler(
                OnProcessStart(
                    target_action=rosbag_process,
                    on_start=filter_madgwick_node,  # Starts when rosbag starts
                )
            ),
        )
    elif IMU_TYPE == "xsens":
        raise NotImplementedError("xsens IMU is not yet supported")

    imu_and_wheel_odom_config_file = os.path.join(
        share_folder, "config", "_imu_and_wheel_odom.yaml"
    )

    imu_and_wheel_odom_node = Node(
        package="norlab_imu_tools",
        executable="imu_and_wheel_odom_node",
        name="imu_and_wheel_odom_node",
        output="log",
        respawn=True,
        parameters=[
            imu_and_wheel_odom_config_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=[
            ("imu_topic", f"{IMU_TYPE}/data"),
            ("wheel_odom_topic", "/warthog/platform/odom"),
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            "warn",
        ],
    )

    mapping_node = Node(
        package="norlab_icp_mapper_ros",
        executable="mapper_node",
        name="mapper_node",
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
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
                    f"_mapper_{LIDAR_TYPE}.yaml",
                ),
                "initial_map_file_name": INPUT_MAP_FILE,
                "initial_robot_pose": "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]",
                "final_map_file_name": f"{OUTPUT_MAP_TRAJ_PATH}/map.vtk",
                "final_trajectory_file_name": f"{OUTPUT_MAP_TRAJ_PATH}/trajectory.vtk",
                "map_publish_rate": 10.0,
                "map_tf_publish_rate": 10.0,
                "max_idle_time": 10.0,
                "is_mapping": IS_MAPPING,
                "is_online": True,
                "is_3D": True,
                "save_map_cells_on_hard_drive": True,
                "publish_tfs_between_registrations": True,
                "deskew": True,
                "compression_voxel_size": 2.5,
            }
        ],
        remappings=[
            ("points_in", f"{LIDAR_TYPE}/points"),
            ("scan_after_input_filters", f"{LIDAR_TYPE}/points_after_input_filters"),
            ("scan_after_deskew", f"{LIDAR_TYPE}/points_after_deskew"),
        ],
    )

    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=rosbag_process,
                on_start=imu_and_wheel_odom_node,
            )
        ),
    )
    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=rosbag_process,
                on_start=mapping_node,  # Starts when rosbag starts
            )
        ),
    )
    return ld
