from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("wheel_robot_4.0", package_name="franzi_moveit_config")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
        )
        .pilz_cartesian_limits()
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
