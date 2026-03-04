from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rita_vision_control',
            executable='yolo_tpu_node',
            name='yolo_tpu_node',
            output='screen',
        )
    ])
