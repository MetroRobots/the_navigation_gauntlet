import argparse
from ament_index_python.packages import get_package_share_path
from ros2launch.api import get_share_file_path_from_package, launch_a_launch_file
import tempfile
import yaml


def write_temp_parameter_file(parameters, node_name='/**'):
    temp_config_path = tempfile.NamedTemporaryFile()
    yaml_data = {node_name: {'ros__parameters': parameters}}
    yaml.safe_dump(yaml_data, open(temp_config_path.name, 'w'))
    return temp_config_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('simulator_package')
    parser.add_argument('nav_config_package')
    args = parser.parse_args()

    record_config = {'topics': []}

    nav_pkg_path = get_package_share_path(args.nav_config_package)
    config_path = nav_pkg_path / 'config/nav_gauntlet.yaml'
    if config_path.exists():
        config = yaml.safe_load(open(config_path))
        record_config['topics'] += config.get('topics', [])

    trial_launch = get_share_file_path_from_package(
        package_name='gauntlet_runner',
        file_name='trial.launch.py')
    launch_arguments = []
    launch_arguments.append(f'simulator_package:={args.simulator_package}')
    launch_arguments.append(f'nav_config_package:={args.nav_config_package}')

    trial_config = {'goal_pose_x': 2.0}
    trial_config_path = write_temp_parameter_file(trial_config)
    launch_arguments.append(f'trial_config_path:={trial_config_path.name}')

    record_config_path = write_temp_parameter_file(record_config)
    launch_arguments.append(f'record_config_path:={record_config_path.name}')

    launch_a_launch_file(
        launch_file_path=trial_launch,
        launch_file_arguments=launch_arguments,
    )

    trial_config_path.close()


if __name__ == '__main__':
    main()
