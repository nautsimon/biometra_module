from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    launch_d = LaunchDescription()
    
    biometra_client = Node(
            package = 'biometra_client',
            namespace = 'std_ns',
            executable = 'biometra_client',
            output = "screen",
            name='biometra_client'
        )
    
    launch_d = LaunchDescription()

    launch_d.add_action(biometra_client)

    return launch_d
