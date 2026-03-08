import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    # 1. Evaluate the use_sim_time argument
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    is_sim = (use_sim_time_str.lower() in ['true', 't', '1', 'y', 'yes'])
    
    # 2. Build MoveIt configs dynamically based on hardware vs sim
    moveit_config = (
        MoveItConfigsBuilder("RITA", package_name="rita_moveit2_config")
        .robot_description(mappings={"use_gazebo": "true" if is_sim else "false"})
        .to_moveit_configs()
    )
    
    servo_params_path = PathJoinSubstitution([
        FindPackageShare('rita_moveit2_config'),
        'config',
        'servo.yaml'
    ])
    
    # --- NODES ---
    yolo_node = Node(
        package='rita_vision_control',
        executable='yolo_tpu_node',
        name='yolo_tpu_node',
        output='screen',
        parameters=[{'use_sim_time': is_sim}]
    )
    
    # Merge MoveIt configs (URDF, SRDF, Kinematics) into a single dictionary for servo_node
    servo_params = moveit_config.to_dict()
    servo_params.update({'use_sim_time': is_sim})
    
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        output='screen',
        parameters=[
            servo_params_path,
            servo_params
        ]
    )
    
    servo_commander_node = Node(
        package='rita_vision_control',
        executable='servo_commander_node', 
        name='servo_commander_node',
        output='screen',
        parameters=[{'use_sim_time': is_sim}]
    )
    
    # --- SEQUENCING ---
    # Delay the commander node by 4 seconds to ensure servo_node is fully initialized
    delayed_commander = TimerAction(
        period=4.0,
        actions=[servo_commander_node]
    )
    
    return [yolo_node, servo_node, delayed_commander]

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # OpaqueFunction executes launch_setup() after arguments are evaluated
    return LaunchDescription([
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])