from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    ld.add_action(
        DeclareLaunchArgument(
            'sim_config_path',
            description='The path to a configuration file for the simulator.'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'robot_name',
            description='The name of the robot in the simulator'
        )
    )

    ld.add_action(IncludeLaunchDescription(
        [FindPackageShare(LaunchConfiguration('simulator_package')), '/launch/simulator_bringup.launch.py'],
        launch_arguments=[
            ('config_path', LaunchConfiguration('sim_config_path')),
            ('robot_name', LaunchConfiguration('robot_name')),
        ]
    ))

    ld.add_action(
        DeclareLaunchArgument(
            'nav_config_package',
            default_value='',
            description='The name of the package from which to launch bringup.launch.py',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'bonus_nav_configuration',
            default_value='',
            description='TODO'
        )
    )

    ld.add_action(IncludeLaunchDescription(
        [FindPackageShare(LaunchConfiguration('nav_config_package')), '/launch/bringup.launch.py'],
        launch_arguments=[
            ('bonus_nav_configuration', LaunchConfiguration('bonus_nav_configuration')),
        ],
        condition=IfCondition(PythonExpression(['len("', LaunchConfiguration('nav_config_package'), '") > 0'])),
    ))

    ld.add_action(
        DeclareLaunchArgument(
            'trial_config_path',
            description='The path to the parameter file for the run_trial node',
            default_value='',
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
            on_exit=EmitEvent(event=Shutdown(reason='run trial node has completed'))
        ),
    ))

    return ld
