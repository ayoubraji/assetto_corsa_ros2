from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simulator_output',
            executable='simulator_output_IAC',
            name='simulator_output_IAC',
            parameters = [
                {'no_opponents': True}
            ],
            output='screen',
            emulate_tty=True
        )
    ])
