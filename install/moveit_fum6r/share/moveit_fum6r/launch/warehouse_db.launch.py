from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fum6r_new_corsys", package_name="moveit_fum6r").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
