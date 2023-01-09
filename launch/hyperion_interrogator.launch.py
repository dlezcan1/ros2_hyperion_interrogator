from launch import LaunchDescription, actions, conditions
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PythonExpression, LocalSubstitution, TextSubstitution, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

pkg_fbg_interrogator = FindPackageShare( 'hyperion_interrogator' )


def generate_launch_description():
    ld = LaunchDescription()

    # arguments
    interrogator_sim_level_arg = DeclareLaunchArgument(
            'sim_level_interrogator',
            default_value="1",
            description="Simulation level: 1 - FBG demo, 2 - real FBG sensors",
    )
    ros_paramfile_arg = DeclareLaunchArgument(
            'paramFile',
            default_value='',
            description='Parameter file to use in local share directory. This overrides all arguments.'
    )
    interrogator_ip_arg = DeclareLaunchArgument(
            'ip',
            default_value='10.0.0.55'
    )
    interrogator_stream_arg = DeclareLaunchArgument(
            "stream_peaks",
            default_value="true",
            description="Whether to use async peak streaming. Must be 'true' or 'false'"
    )
    needle_paramfile_arg = DeclareLaunchArgument(
            'needleParamFile',
            default_value='',
            description="needle parameter JSON file for loading the persistent reference wavelengths",
    )

    num_samples_arg = DeclareLaunchArgument(
            'numSamples',
            default_value='200'
    )
    demo_num_chs_arg = DeclareLaunchArgument( 'numCH', default_value="3" )
    demo_num_aas_arg = DeclareLaunchArgument( 'numAA', default_value="4" )

    # launch descriptions
    ld_demo_interrogator = IncludeLaunchDescription(  # demo FBG interrogator
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution( [ pkg_fbg_interrogator, 'hyperion_demo.launch.py' ] )
            ),
            condition=conditions.LaunchConfigurationEquals( 'sim_level_interrogator', "1" ),
            launch_arguments={
                    'ip'            : LaunchConfiguration( 'interrogatorIP' ),
                    'numCH'         : LaunchConfiguration( 'numCH' ),
                    'numAA'         : LaunchConfiguration( 'numAA' ),
                    'paramFile'     : LaunchConfiguration( 'paramFile' ),
            }.items(),
    )

    # FBG Interrogator
    ld_fbg_interrogator_sync = IncludeLaunchDescription(  # real sync FBG interrogator
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution( [ pkg_fbg_interrogator, 'hyperion_talker.launch.py' ] )
            ),
            condition=conditions.IfCondition(
                    PythonExpression(
                            [
                                    "'", interrogator_sim_level_arg, "' == '2'",
                                    " and ",
                                    "'", interrogator_stream_arg, "' != 'true'"

                            ]
                    )
            ),
            launch_arguments={
                    'ip'             : LaunchConfiguration( 'interrogatorIP' ),
                    'paramFile'      : LaunchConfiguration( 'paramFile' ),
                    'needleParamFile': LaunchConfiguration( 'needleParamFile' )
            }.items()
    )
    ld_fbg_interrogator_sync = IncludeLaunchDescription(  # real async FBG interrogator
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution( [ pkg_fbg_interrogator, 'hyperion_streamer.launch.py' ] )
            ),
            condition=conditions.IfCondition(
                    PythonExpression(
                            [
                                    "'", interrogator_sim_level_arg, "' == '2'",
                                    " and ",
                                    "'", interrogator_stream_arg, "' != 'true'"

                            ]
                    )
            ),
            launch_arguments={
                    'ip'             : LaunchConfiguration( 'interrogatorIP' ),
                    'paramFile'      : LaunchConfiguration( 'paramFile' ),
                    'needleParamFile': LaunchConfiguration( 'needleParamFile' )
            }.items()
    )

    # configure launch description
    ld.add_action( interrogator_sim_level_arg )

    ld.add_action( ros_paramfile_arg )
    ld.add_action( interrogator_ip_arg )
    ld.add_action( num_samples_arg )
    ld.add_action( interrogator_stream_arg )
    ld.add_action( needle_paramfile_arg )

    ld.add_action( demo_num_chs_arg )
    ld.add_action( demo_num_aas_arg )

    ld.add_action( ld_demo_interrogator )
    ld.add_action( ld_fbg_interrogator_sync )
    ld.add_action( ld_fbg_interrogator_async )

    return ld

# generate_launch_description
