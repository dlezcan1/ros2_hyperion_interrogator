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
    demo_num_chs_arg = DeclareLaunchArgument('numCH', default_value="3")
    demo_num_aa_arg  = DeclareLaunchArgument('numAA', default_value="4")
    
    # Nodes
    hyperion_node = Node(
                        package='hyperion_interrogator',
                        namespace='needle',
                        executable='hyperion_demo',
                        parameters=[{
                                    "interrogator.ip_address": LaunchConfiguration('ip'),
                                    "demo.num_channels": LaunchConfiguration("numCH"),
                                    "demo.num_active_areas": LaunchConfiguration("numAA")
                                    }]
                       )
    # add to launch description
    ld.add_action(hyperion_ip_arg)
    ld.add_action(demo_num_chs_arg)
    ld.add_action(demo_num_aa_arg)
    ld.add_action(hyperion_node)
       
    return ld
     
 # generate_launch_description
