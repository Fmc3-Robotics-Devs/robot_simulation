from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = Path(get_package_share_directory("franzi_moveit_config")) / "launch"

    use_rviz = LaunchConfiguration("use_rviz")
    allow_execution = LaunchConfiguration("allow_trajectory_execution")
    publish_demo_joint_states = LaunchConfiguration("publish_demo_joint_states")
    publish_chassis_joints = LaunchConfiguration("publish_chassis_joints")
    publish_virtual_joint_tf = LaunchConfiguration("publish_virtual_joint_tf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "allow_trajectory_execution",
                default_value="true",
                description="Enable execution through the demo's virtual controllers.",
            ),
            DeclareLaunchArgument(
                "publish_demo_joint_states",
                default_value="true",
                description=(
                    "Publish initial joint states for the planning-only demo. "
                    "Set false when a controller or simulator publishes /joint_states."
                ),
            ),
            DeclareLaunchArgument(
                "publish_chassis_joints",
                default_value="true",
                description=(
                    "Include the steering and wheel joints in the demo joint states. "
                    "Set false when a base driver owns them."
                ),
            ),
            DeclareLaunchArgument(
                "publish_virtual_joint_tf",
                default_value="true",
                description=(
                    "Publish an identity world -> moveit_root transform, which pins "
                    "the chassis at the origin. Set false when a base driver "
                    "publishes the base pose."
                ),
            ),
            Node(
                package="franzi_moveit_config",
                executable="demo_joint_state_publisher.py",
                name="demo_robot_driver",
                output="screen",
                parameters=[{"publish_chassis_joints": publish_chassis_joints}],
                condition=IfCondition(publish_demo_joint_states),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(launch_dir / "static_virtual_joint_tfs.launch.py")
                ),
                condition=IfCondition(publish_virtual_joint_tf),
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
