import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    is_sim = False
    
    # Build MoveIt configs dynamically
    moveit_config = (
        MoveItConfigsBuilder("RITA", package_name="rita_moveit2_config")
        .robot_description(mappings={"use_gazebo": "false"})
        .to_moveit_configs()
    )
    
    # 1. Hardware Interface (ros2_control) - Now talks directly to PC's USB port!
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(moveit_config.package_path, "config", "ros2_controllers.yaml"),
        ],
        output="screen",
    )
    
    # 2. Robot State Publisher (TF Tree)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    
    # 3. Spawners (Load the physical motors)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    rita_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rita_arm_controller", "--controller-manager", "/controller_manager"],
    )
    
    # Delay arm controller until broadcaster is active
    delay_rita_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rita_controller_spawner],
        )
    )
    
    # 4. MoveIt Servo Node (The Reflexes)
    servo_params_path = PathJoinSubstitution([
        FindPackageShare('rita_moveit2_config'),
        'config',
        'servo.yaml'
    ])
    servo_params = moveit_config.to_dict()
    servo_params.update({'use_sim_time': is_sim})
    
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        output='screen',
        parameters=[servo_params_path, servo_params]
    )
    
    # 5. YOLO WebRTC Node (The Brains - reading from the Pi's stream)
    yolo_node = Node(
        package='rita_vision_control',
        executable='yolo_webrtc_node', 
        name='yolo_webrtc_node',
        output='screen'
    )
    
    # 6. Servo Commander Node (The Logic)
    servo_commander_node = Node(
        package='rita_vision_control',
        executable='servo_commander_node', 
        name='servo_commander_node',
        output='screen',
        parameters=[{'use_sim_time': is_sim}]
    )
    
    # Delay the commander to give MoveIt Servo time to boot
    delayed_commander = TimerAction(
        period=4.0,
        actions=[servo_commander_node]
    )
    
    # 7. RViz2 (The Visualization)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(str(moveit_config.package_path), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': is_sim}
        ],
    )
    
    return [
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_rita_controller_spawner,
        servo_node,
        yolo_node,
        delayed_commander,
        rviz_node
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])