from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            'simulator_package',
            description='The name of the package from which to launch simulator_bringup.launch.py'
        )
    )

    ld.add_action(IncludeLaunchDescription(
        [FindPackageShare(LaunchConfiguration('simulator_package')), '/launch/simulator_bringup.launch.py'],
    ))

    ld.add_action(
        DeclareLaunchArgument(
            'nav_config_package',
            description='The name of the package from which to launch bringup.launch.py'
        )
    )

    ld.add_action(IncludeLaunchDescription(
        [FindPackageShare(LaunchConfiguration('nav_config_package')), '/launch/bringup.launch.py'],
    ))

    ld.add_action(
        DeclareLaunchArgument(
            'trial_config_path',
            description='The path to the parameter file for the run_trial node'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'record_config_path',
            default_value='',
            description='The path to a configuration file for the bag recorder.'
        )
    )
    ld.add_action(Node(
        package='gauntlet_runner',
        executable='recorder',
        parameters=[LaunchConfiguration('record_config_path')],
        output='screen',
    ))

    run_trial_node = Node(
        package='gauntlet_runner',
        executable='run_trial',
        parameters=[LaunchConfiguration('trial_config_path')],
        output='screen',
    )

    ld.add_action(run_trial_node)

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=run_trial_node,
            on_exit=EmitEvent(event=Shutdown())
        ),
    ))

    return ld
