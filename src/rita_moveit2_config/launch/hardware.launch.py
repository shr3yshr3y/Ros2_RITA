import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load MoveIt configs, which will read your updated RITA.ros2_control.xacro
    moveit_config = (
        MoveItConfigsBuilder("RITA", package_name="rita_moveit2_config")
        .robot_description(mappings={"use_gazebo": "false"})
        .to_moveit_configs()
    )
    # Node to start the actual ros2_control framework with your hardware interface
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
        output="both",
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawn the RITA joint trajectory controller
    rita_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rita_arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay the start of the arm controller until the state broadcaster finishes loading
    delay_rita_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rita_controller_spawner],
        )
    )

    # --- NEW: Start the MoveIt 2 Brain ---
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz to interact with MoveIt
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
        ],
    )

    return LaunchDescription(
        [
            control_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            delay_rita_controller_spawner_after_joint_state_broadcaster_spawner,
            run_move_group_node,  # Added here!
            rviz_node,
        ]
    )