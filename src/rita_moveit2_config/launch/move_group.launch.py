from launch import LaunchDescription
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("RITA", package_name="rita_moveit2_config").to_moveit_configs()
    
    # We unpack the generated launch description and inject the use_sim_time parameter globally
    ld = generate_move_group_launch(moveit_config)
    
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        *ld.entities
    ])