from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speech_output_server',
            executable='speech_output_action_server'
        )
    ])