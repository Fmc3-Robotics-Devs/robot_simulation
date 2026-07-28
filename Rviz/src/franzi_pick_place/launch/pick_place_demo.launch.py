"""Bring up the MoveIt demo plus the machine-tending task.

The MoveIt demo supplies move_group, the robot state publisher and the virtual
trajectory controllers; this file adds the RViz config that also shows the
station status light, and the task node itself.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_launch = Path(get_package_share_directory("franzi_moveit_config")) / "launch"
    task_share = Path(get_package_share_directory("franzi_pick_place"))

    use_rviz = LaunchConfiguration("use_rviz")
    task_config = LaunchConfiguration("task_config")
    start_delay = LaunchConfiguration("start_delay")

    task_node = Node(
        package="franzi_pick_place",
        executable="pick_place_task",
        name="pick_place_task",
        output="screen",
        parameters=[task_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "task_config",
                default_value=str(task_share / "config" / "task.yaml"),
                description="Cell layout and task parameters.",
            ),
            DeclareLaunchArgument(
                "start_delay",
                default_value="6.0",
                description="Seconds to wait for move_group before the task starts.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(moveit_launch / "demo.launch.py")),
                launch_arguments={"use_rviz": "false"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(moveit_launch / "moveit_rviz.launch.py")),
                launch_arguments={
                    "rviz_config": str(task_share / "config" / "pick_place.rviz")
                }.items(),
                condition=IfCondition(use_rviz),
            ),
            TimerAction(period=start_delay, actions=[task_node]),
        ]
    )
