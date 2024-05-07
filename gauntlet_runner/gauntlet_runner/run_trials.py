import argparse
from ament_index_python.packages import get_package_share_path, PackageNotFoundError
from ros2launch.api import get_share_file_path_from_package, launch_a_launch_file
from .exploration import explore_parameter_space
import tempfile
import pathlib
import yaml


def get_nav_gauntlet_params(package_name):
    try:
        pkg_path = get_package_share_path(package_name)
        config_path = pkg_path / 'config/nav_gauntlet.yaml'
        if config_path.exists():
            return yaml.safe_load(open(config_path))
    except PackageNotFoundError:
        pass
    return {}


def write_temp_parameter_file(parameters, node_name='/**'):
    temp_config_path = tempfile.NamedTemporaryFile()
    yaml_data = {node_name: {'ros__parameters': parameters}}
    yaml.safe_dump(yaml_data, open(temp_config_path.name, 'w'))
    return temp_config_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('trials_config_path')
    parser.add_argument('-f', '--force', action='store_true', help='overwrite existing data')
    args = parser.parse_args()

    trial_launch = get_share_file_path_from_package(
        package_name='gauntlet_runner',
        file_name='trial.launch.py')

    trials_config = yaml.safe_load(open(args.trials_config_path))
    record_path = trials_config.get('record_path', 'NavTrialBag')

    for trial_config in explore_parameter_space(trials_config['parameters']):
        trial_record_path = pathlib.Path(record_path.format(**trial_config)).expanduser().resolve()
        if trial_record_path.exists():
            if args.force:
                trial_record_path.unlink()
            else:
                print(f'Skipping {trial_record_path}')
        trial_record_path.parent.mkdir(exist_ok=True, parents=True)

        launch_arguments = []
        record_config = {'topics': []}
        record_config['record_path'] = str(trial_record_path)

        simulator_pkg = trial_config['simulator_pkg']
        launch_arguments.append(f'simulator_package:={simulator_pkg}')
        sim_config = get_nav_gauntlet_params(simulator_pkg)
        record_config['topics'] += sim_config.get('topics', [])

        trial_config_path = write_temp_parameter_file(trial_config)
        launch_arguments.append(f'trial_config_path:={trial_config_path.name}')

        record_config_path = write_temp_parameter_file(record_config)
        launch_arguments.append(f'record_config_path:={record_config_path.name}')

        launch_a_launch_file(
            launch_file_path=trial_launch,
            launch_file_arguments=launch_arguments,
        )

        trial_config_path.close()
        record_config_path.close()


if __name__ == '__main__':
    main()
