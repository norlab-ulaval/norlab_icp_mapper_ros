import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    share_folder = get_package_share_directory("norlab_icp_mapper_ros")
    imu_and_wheel_odom_config_file = os.path.join(
        share_folder, "config", "_imu_and_wheel_odom.yaml"
    )

    imu_and_wheel_odom_node = Node(
        package="norlab_imu_tools",
        executable="imu_and_wheel_odom_node",
        name="imu_and_wheel_odom_node",
        output="log",
        respawn=False,
        parameters=[
            imu_and_wheel_odom_config_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=[
            ("imu_topic", "/vn100/data"),
            ("wheel_odom_topic", "/warthog/platform/odom"),
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
                    "mapper_rs.yaml",
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
                "compression_voxel_size": 1.5,
            }
        ],
        remappings=[
            ("points_in", "rslidar128/points"),
            ("scan_after_input_filters", "rslidar128/points_after_input_filters"),
            ("scan_after_deskew", "rslidar128/points_after_deskew"),
        ],
    )
    robot_name = "warthog"
    if robot_name is None:
        raise EnvironmentError("ROBOT_NAME environment variable is not set")

    # Get URDF via xacro
    package = get_package_share_directory("norlab_robot")
    xacro_path = os.path.join(package, "urdf/main.urdf.xacro")

    robot_description_command = Command(["xacro ", xacro_path, f" name:={robot_name}"])
    robot_description = {
        "robot_description": ParameterValue(robot_description_command, value_type=str)
    }

    is_warthog = os.environ.get("IS_WARTHOG", None)
    remappings = []
    if is_warthog == "1":
        remappings = [("tf_static", "dummy_tf_static"), ("tf", "dummy_tf")]

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=remappings,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation time"
            ),
            imu_and_wheel_odom_node,
            mapping_node,
            robot_state_publisher_node,
        ]
    )
