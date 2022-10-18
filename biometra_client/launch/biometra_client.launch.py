from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_d = LaunchDescription()
    
    biometra_client = Node(
            package = 'biometra_client',
            namespace = 'biometra_client',
            executable = 'biometra_client',
            output = "screen",
            name='biometraNode'
    )

    launch_d.add_action(biometra_client)
    return launch_d
    