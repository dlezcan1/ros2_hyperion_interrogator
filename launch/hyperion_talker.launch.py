from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # arguments
    hyperion_ip_arg = DeclareLaunchArgument( 'ip',
                                             default_value='10.0.0.55'
                                             )
    num_samples_arg = DeclareLaunchArgument( 'numSamples',
                                             default_value='200'
                                             )

    # Nodes
    hyperion_node = Node(
            package='hyperion_interrogator',
            namespace='needle',
            executable='hyperion_talker',
            parameters=[ {
                    "interrogator.ip_address": LaunchConfiguration( 'ip' ),
                    "sensor.num_samples"     : LaunchConfiguration( 'numSamples' ),
                    } ]
            )
    # add to launch description
    ld.add_action( hyperion_ip_arg )
    ld.add_action( num_samples_arg )
    ld.add_action( hyperion_node )

    return ld

# generate_launch_description
