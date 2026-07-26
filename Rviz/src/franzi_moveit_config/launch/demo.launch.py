from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = Path(get_package_share_directory("franzi_moveit_config")) / "launch"

    use_rviz = LaunchConfiguration("use_rviz")
    allow_execution = LaunchConfiguration("allow_trajectory_execution")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "allow_trajectory_execution",
                default_value="false",
                description="No robot controller is configured; keep false for this demo.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(launch_dir / "static_virtual_joint_tfs.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_dir / "rsp.launch.py"))
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_dir / "move_group.launch.py")),
                launch_arguments={
                    "allow_trajectory_execution": allow_execution,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(launch_dir / "moveit_rviz.launch.py")
                ),
                condition=IfCondition(use_rviz),
            ),
        ]
    )
