from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    ld = LaunchDescription()
    
    # arguments
    hyperion_ip_arg = DeclareLaunchArgument('ip',
                                            default_value='10.0.0.55'
                                            )
    
    # Nodes
    hyperion_node = Node(
                        package='hyperion_interrogator',
                        namespace='needle',
                        executable='hyperion_talker',
                        parameters=[{
                                    "interrogator.ip_address": LaunchConfiguration('ip')
                                    }]
                       )
    # add to launch description
    ld.add_action(hyperion_ip_arg)
    ld.add_action(hyperion_node)
       
    return ld
     
 # generate_launch_description
