from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bobo_description',
            executable='mapping',
            name='mapping_mode',
            output='screen'
        )
    ])
