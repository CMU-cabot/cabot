from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cabot_ui',
            executable='cabot_keyboard.py',
            name='cabot_keyboard_node',
            output='screen',
            emulate_tty=True,
            shell=True
        )
    ])
