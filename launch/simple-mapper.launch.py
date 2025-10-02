import os
import json
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription

IS_MAPPING = os.getenv("IS_MAPPING")
STORAGE_PATH = os.getenv("STORAGE_PATH")
INPUT_IMU_BIAS_FILE = os.path.join("/", "calib", "imu.json")
IMU_TYPE = "vectornav"  # or 'xsens'
LIDAR_TYPE = "robosense"

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

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation time"
        )
    )
    bias_x = 0.0
    bias_y = 0.0
    bias_z = 0.0

    if os.path.exists(INPUT_IMU_BIAS_FILE):
        with open(INPUT_IMU_BIAS_FILE, "r") as f:
            bias_data = json.load(f)
            bias_x = bias_data[IMU_TYPE]["angular_velocity"]["x"]
            bias_y = bias_data[IMU_TYPE]["angular_velocity"]["y"]
            bias_z = bias_data[IMU_TYPE]["angular_velocity"]["z"]
    else:
        print("No bias file found, using default values")

    if IMU_TYPE == "vectornav":
        namespace = LaunchConfiguration("vn100_ns")
        vectornav_namespace_launch_arg = DeclareLaunchArgument(
            "vn100_ns", default_value=IMU_TYPE
        )

        config_file = os.path.join(share_folder, "config", "_vn100.yaml")

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
        ld.add_action(bias_compensator_node)
        ld.add_action(filter_madgwick_node)
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
        sigterm_timeout="30",  # Wait 30 seconds before escalating to SIGTERM
        sigkill_timeout="5",  # Wait 5 more seconds before SIGKILL
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
                    f"_mapper_{LIDAR_TYPE}.yaml",
                ),
                "initial_map_file_name": input_map_name,
                "initial_robot_pose": "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]",
                "final_map_file_name": output_map_name,
                "final_trajectory_file_name": "trajectory.vtk",
                "map_publish_rate": 10.0,
                "map_tf_publish_rate": 10.0,
                "max_idle_time": 10.0,
                "is_mapping": IS_MAPPING,
                "is_online": True,
                "is_3D": True,
                "save_map_cells_on_hard_drive": True,
                "publish_tfs_between_registrations": True,
                "deskew": False,
                "compression_voxel_size": 0.5,
            }
        ],
        remappings=[
            ("points_in", f"{LIDAR_TYPE}/points"),
            ("scan_after_input_filters", f"{LIDAR_TYPE}/points_after_input_filters"),
            ("scan_after_deskew", f"{LIDAR_TYPE}/points_after_deskew"),
            ("icp_odom", "estimated_odom"),
        ],
    )

    ld.add_action(imu_and_wheel_odom_node)
    ld.add_action(mapping_node)
    return ld
