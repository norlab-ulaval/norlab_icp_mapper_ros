import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

INPUT_IMU_BIAS_FILE = os.path.join("/", "calib", "imu.json")
IMU_TYPE = os.getenv("IMU_TYPE", "vectornav")  # or 'xsens'

NAMESPACE = os.getenv("NAMESPACE")


def generate_launch_description():
    ld = LaunchDescription()
    share_folder = get_package_share_directory("ros_launchers")

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
            print(f"Biases: x={bias_x}, y={bias_y}, z={bias_z}")
    else:
        print("No bias file found, using default values")

    config_file = os.path.join(share_folder, "config", "_imu.yaml")

    bias_compensator_node = Node(
        package="norlab_imu_tools",
        executable="imu_bias_compensator_node",
        name="bias_compensator",
        namespace=NAMESPACE,
        parameters=[
            config_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "bias_x": bias_x,
                "bias_y": bias_y,
                "bias_z": bias_z,
            },
        ],
        remappings=[
            ("imu_topic_in", f"/{IMU_TYPE}/data_raw"),
            ("bias_topic_in", f"/{IMU_TYPE}/bias"),
            ("imu_topic_out", f"{IMU_TYPE}/data_unbiased"),
            ("/tf", "tf"),
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
        namespace=NAMESPACE,
        output="both",
        parameters=[
            config_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=[
            ("imu/data_raw", f"{IMU_TYPE}/data_unbiased"),
            ("imu/mag", f"{IMU_TYPE}/mag"),
            ("imu/data", f"{IMU_TYPE}/data"),
            ("/tf", "tf"),
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            "warn",
        ],
    )
    ld.add_action(bias_compensator_node)
    ld.add_action(filter_madgwick_node)

    return ld
