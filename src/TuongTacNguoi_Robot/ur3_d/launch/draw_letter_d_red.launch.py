from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3",
                description="UR robot type to simulate.",
            ),
            DeclareLaunchArgument(
                "writer_delay",
                default_value="8.0",
                description="Delay after simulation readiness before drawing starts.",
            ),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("startup_timeout", default_value="90.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("ur3_d"), "/launch/ur3_d.launch.py"]
                ),
                launch_arguments={
                    "ur_type": LaunchConfiguration("ur_type"),
                    "start_writer": "true",
                    "writer_delay": LaunchConfiguration("writer_delay"),
                    "writer_config_file": "writer_vertical_red.yaml",
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                    "startup_timeout": LaunchConfiguration("startup_timeout"),
                }.items(),
            ),
        ]
    )
