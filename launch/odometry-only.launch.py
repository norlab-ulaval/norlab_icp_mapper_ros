import os
import json
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription

INPUT_PATH = "/"

IS_MAPPING = True
INPUT_IMU_BIAS_FILE = os.path.join(INPUT_PATH, "calib", "imu.json")
IMU_TYPE = "vectornav"  # or 'xsens'
LIDAR_TYPE = "robosense"


def generate_launch_description():
    ld = LaunchDescription()
    share_folder = get_package_share_directory("norlab_icp_mapper_ros")

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation time"
        )
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
            ("imu_and_wheel_odom", "estimated_odom"),
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            "warn",
        ],
    )

    ld.add_action(imu_and_wheel_odom_node)
    return ld
