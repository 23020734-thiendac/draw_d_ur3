from uuid import uuid4

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription, LogInfo,
    RegisterEventHandler, SetEnvironmentVariable, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    start_writer = LaunchConfiguration("start_writer")
    writer_delay = LaunchConfiguration("writer_delay")
    writer_config_file = LaunchConfiguration("writer_config_file")
    # Gazebo transport is independent of ROS_DOMAIN_ID. Never reuse an old
    # server's /world/empty/create service or its simulation clock.
    gazebo_partition = "ur3_d_" + uuid4().hex
    kinematics_yaml = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )
    writer_yaml = PathJoinSubstitution(
        [FindPackageShare("ur3_d"), "config", writer_config_file]
    )

    ur_sim_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("ur_simulation_gz"),
                "/launch/ur_sim_control.launch.py",
            ]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "false",
            # Run the server separately. The combined Fortress server/GUI
            # launcher can leave a forked server alive after Ctrl+C.
            "gazebo_gui": "false",
        }.items(),
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": "-g -v 2"}.items(),
        condition=IfCondition(LaunchConfiguration("gazebo_gui")),
    )

    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("ur3_d"),
                "/launch/moveit_rviz.launch.py",
            ]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_servo": "false",
        }.items(),
    )

    write_letter_node = TimerAction(
        period=writer_delay,
        actions=[
            Node(
                package="ur3_d",
                executable="write_letter_d_node",
                name="write_letter_d_node",
                output="screen",
                parameters=[
                    writer_yaml,
                    kinematics_yaml,
                    {
                        "use_sim_time": True,
                    }
                ],
            )
        ],
        condition=IfCondition(start_writer),
    )

    preflight = Node(
        package="ur3_d", executable="simulation_ready.py",
        arguments=["--preflight"], output="screen",
    )
    ready = Node(
        package="ur3_d", executable="simulation_ready.py",
        arguments=["--timeout", LaunchConfiguration("startup_timeout")],
        output="screen",
    )

    def after_preflight(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="UR3 preflight failed; see error above."))]
        # The upstream include sets launch_rviz=false. Keep its arguments local
        # so it cannot overwrite the user's RViz setting for the MoveIt include.
        return [
            GroupAction(actions=[ur_sim_control]),
            GroupAction(actions=[gazebo_gui]),
            ready,
        ]

    def after_ready(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="UR3 simulation is not ready; see error above."))]
        return [GroupAction(actions=[ur_moveit]), write_letter_node]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3",
                description="UR robot type to simulate.",
            ),
            DeclareLaunchArgument(
                "start_writer",
                default_value="false",
                description=(
                    "Start the writer automatically; keep false for the "
                    "two-command workflow."
                ),
            ),
            DeclareLaunchArgument(
                "writer_delay",
                default_value="25.0",
                description="Delay after simulation readiness before starting the writer node.",
            ),
            DeclareLaunchArgument(
                "writer_config_file",
                default_value="writer.yaml",
                description="Writer parameter file inside ur3_d/config.",
            ),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("startup_timeout", default_value="90.0"),
            SetEnvironmentVariable("IGN_PARTITION", gazebo_partition),
            SetEnvironmentVariable("GZ_PARTITION", gazebo_partition),
            LogInfo(msg="Gazebo transport partition: " + gazebo_partition),
            RegisterEventHandler(OnProcessExit(target_action=preflight, on_exit=after_preflight)),
            RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=after_ready)),
            preflight,
        ]
    )
