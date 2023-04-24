from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    launch_d = LaunchDescription()
    
    robot_name = LaunchConfiguration('robot_name')
    device_name =  LaunchConfiguration('device_name')
    
    declare_use_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value="biometra96"
        description='Flag to accept robot_name')
    
    declare_use_robot_name_cmd = DeclareLaunchArgument(
        name='device_name',
        default_value="device1"
        description='Flag to accept robot_name')
    
    biometra_client = Node(
            package = 'biometra_client',
            namespace = 'std_ns',
            executable = 'biometra_client',
            output = "screen",
            name = robot_name,
            parameters = [
                {"device_name":device_name}
                ],
            emulate_tty = True,        
            )
    
    launch_d = LaunchDescription()
    launch_d.add_action(declare_use_robot_name_cmd)
    launch_d.add_action(declare_use_robot_name_cmd)
    launch_d.add_action(biometra_client)

    return launch_d
