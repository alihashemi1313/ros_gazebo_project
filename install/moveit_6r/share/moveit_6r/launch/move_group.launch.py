from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fum6r_new_corsys", package_name="moveit_6r").to_moveit_configs()
    return generate_move_group_launch(moveit_config)