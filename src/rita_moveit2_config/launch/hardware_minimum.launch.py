import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load basic configs (URDF/SRDF needed for TF and motor controllers)
    moveit_config = (
        MoveItConfigsBuilder("RITA", package_name="rita_moveit2_config")
        .robot_description(mappings={"use_gazebo": "false"})
        .to_moveit_configs()
    )

    # 2. Start the ros2_control framework (Talks to the physical hardware)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                moveit_config.package_path,
                "config",
                "ros2_controllers.yaml",
            ),
        ],
        output="screen",
    )

    # 3. Robot State Publisher (Publishes the physical joints to the TF tree)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # 4. Pi Camera Node
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'width': 640,
            'height': 480,
            'fps': 30.0,
            'format': 'RGB888',
            'camera_name': 'camera',
            'frame_id': 'camera_link',
        }],
        remappings=[
            ('~/image_raw', '/camera/image_raw'),
            ('~/camera_info', '/camera/camera_info'),
            ('~/image_raw/compressed', '/camera/image_raw/compressed')
        ]
    )

    # 5. Spawn joint state broadcaster (Reads motor angles and publishes /joint_states)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 6. Spawn the RITA hardware controller (Listens for MoveIt Servo commands)
    rita_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rita_arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay the start of the arm controller until the broadcaster is successfully up
    delay_rita_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rita_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            control_node,
            robot_state_publisher,
            camera_node,
            joint_state_broadcaster_spawner,
            delay_rita_controller_spawner,
        ]
    )