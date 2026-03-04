import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    # File Paths
    pkg_rita_gazebo = get_package_share_directory('rita_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_rita_moveit2_config = get_package_share_directory('rita_moveit2_config') 
    
    # Point to the MoveIt Xacro file instead of the raw URDF
    urdf_file = os.path.join(pkg_rita_moveit2_config, 'config', 'RITA.urdf.xacro')
    world_file = os.path.join(pkg_rita_gazebo, 'worlds', 'empty.world')

    # Parse URDF using Command and pass use_gazebo:=true
    robot_description_content = Command(['xacro ', urdf_file, ' use_gazebo:=true'])
    robot_description = {'robot_description': robot_description_content, 'use_sim_time': True}

    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. Start Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 3. Spawn Robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'rita', '-allow_renaming', 'true']
    )

    # 4. Clock and Camera Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.JointState', 
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # 5. Spawners for Controllers
    spawn_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}]
    )

    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rita_arm_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}]
    )

# 6. MoveIt 2 and RViz
    move_group_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rita_moveit2_config, 'launch', 'move_group.launch.py') # <-- Fixed name here
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rita_moveit2_config, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # --- SEQUENCING ---
    # Ensure controllers spawn AFTER the robot is successfully loaded in Gazebo
    delay_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[spawn_broadcaster],
        )
    )
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_broadcaster,
            on_exit=[spawn_arm_controller],
        )
    )
    
    # Ensure MoveIt and RViz spawn AFTER the arm controller is up
    delay_moveit_and_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_arm_controller,
            on_exit=[move_group_node, rviz_node],
        )
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        delay_broadcaster,
        delay_arm_controller,
        delay_moveit_and_rviz
    ])