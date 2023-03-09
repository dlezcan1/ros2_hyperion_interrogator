from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # arguments
    hyperion_ip_arg = DeclareLaunchArgument(
            'ip',
            default_value='10.0.0.55'
    )
    num_samples_arg = DeclareLaunchArgument(
            'numSamples',
            default_value='200'
    )
    needle_paramfile_arg = DeclareLaunchArgument(
            'needleParamFile',
            default_value='',
            description="needle parameter JSON file for loading the persistent reference wavelengths",
    )
    demo_num_chs_arg = DeclareLaunchArgument( 'numCH', default_value="3" )
    demo_num_aa_arg = DeclareLaunchArgument( 'numAA', default_value="4" )

    # Nodes
    hyperion_node = Node(
            package='hyperion_interrogator',
            namespace='needle',
            executable='hyperion_demo',
            output='screen',
            emulate_tty=True,
            parameters=[ {
                    "interrogator.ip_address": LaunchConfiguration( 'ip' ),
                    "sensor.num_samples"     : LaunchConfiguration( 'numSamples' ),
                    "fbg_needle.path"        : LaunchConfiguration( 'needleParamFile' ),
                    "demo.num_channels"      : LaunchConfiguration( "numCH" ),
                    "demo.num_active_areas"  : LaunchConfiguration( "numAA" ),
            } ]
    )
    # add to launch description
    ld.add_action( hyperion_ip_arg )
    ld.add_action( num_samples_arg )
    ld.add_action( needle_paramfile_arg )

    ld.add_action( demo_num_chs_arg )
    ld.add_action( demo_num_aa_arg )

    ld.add_action( hyperion_node )

    return ld

# generate_launch_description
