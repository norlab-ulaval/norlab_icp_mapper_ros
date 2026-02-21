import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

IMU_TYPE = os.getenv("IMU_TYPE", "vectornav")  # or 'xsens'
NAMESPACE = os.getenv("NAMESPACE", "")


def generate_launch_description():
    ld = LaunchDescription()
    share_folder = get_package_share_directory("ros_launchers")
    launch_folder = os.path.join(share_folder, "launch")

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation time"
        )
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_folder, "imu.launch.py")])
    )
    ld.add_action(imu_launch)

    config_file = os.path.join(share_folder, "config", "_imu_and_wheel_odom.yaml")

    imu_and_wheel_odom_node = Node(
        package="norlab_imu_tools",
        executable="imu_and_wheel_odom_node",
        name="imu_and_wheel_odom_node",
        namespace=NAMESPACE,
        output="log",
        respawn=True,
        parameters=[
            config_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=[
            ("imu_topic", f"{IMU_TYPE}/data"),
            ("wheel_odom_topic", "/warthog/platform/odom"),
            ("imu_and_wheel_odom", "estimated_odom"),
            ("/tf", f"/{NAMESPACE}/tf"),
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            "warn",
        ],
    )

    ld.add_action(imu_and_wheel_odom_node)
    return ld
