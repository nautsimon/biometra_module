from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='biometra_client',
            namespace='biometra_module',
            executable='biometraNode',
            name='biometraNode'
        ),
    ])
