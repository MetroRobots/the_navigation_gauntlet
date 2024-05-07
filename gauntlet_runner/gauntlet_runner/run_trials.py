import argparse
from ament_index_python.packages import get_package_share_path, PackageNotFoundError
from ros2launch.api import get_share_file_path_from_package, launch_a_launch_file
from .exploration import explore_parameter_space, format_value
import tempfile
import pathlib
import yaml


def get_record_path(root_path, trials_config, param_config):
    bag_path = root_path
    for parameter in trials_config['parameters']:
        full_pname = parameter['name']
        value = param_config[full_pname]
        bag_path = bag_path / format_value(parameter, value)
    return bag_path


def get_parameters(trials_config, param_config):
    sim_config = dict(trials_config.get('sim', {}))
    data_config = dict(trials_config.get('data', {}))
    trial_config = dict(trials_config.get('trial', {}))

    for parameter in trials_config['parameters']:
        full_pname = parameter['name']
        ns, _, name = full_pname.partition('/')
        value = param_config[full_pname]

        if ns == 'sim':
            sim_config[name] = value
        elif ns == 'data':
            data_config[name] = value
        elif ns == 'trial':
            trial_config[name] = value
        else:
            raise RuntimeError(f'Unknown namespace {ns}')

    if 'topics' not in data_config:
        data_config['topics'] = []

    return sim_config, data_config, trial_config


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
    parser.add_argument('trials_config_path', type=pathlib.Path)
    parser.add_argument('-f', '--force', action='store_true', help='overwrite existing data')
    args = parser.parse_args()

    trial_launch = get_share_file_path_from_package(
        package_name='gauntlet_runner',
        file_name='trial.launch.py')

    trials_config = yaml.safe_load(open(args.trials_config_path))
    if 'root_path' in trials_config:
        root_path = pathlib.Path(trials_config['root_path'])
    else:
        root_path = args.trials_config_path.parent

    root_path = root_path.expanduser().resolve()

    for param_config in explore_parameter_space(trials_config['parameters']):
        bag_path = get_record_path(root_path, trials_config, param_config)
        bag_path.parent.mkdir(exist_ok=True, parents=True)

        if bag_path.exists():
            if args.force:
                bag_path.unlink()
            else:
                print(f'Skipping {bag_path}')
                continue

        sim_config, data_config, trial_config = get_parameters(trials_config, param_config)
        launch_arguments = []

        # Load simulator args
        simulator_pkg = sim_config['pkg']
        launch_arguments.append(f'simulator_package:={simulator_pkg}')
        sim_pkg_config = get_nav_gauntlet_params(simulator_pkg)
        data_config['topics'] += sim_pkg_config.get('topics', [])

        trial_sim_config_path = write_temp_parameter_file(simulator_pkg)
        launch_arguments.append(f'sim_config_path:={trial_sim_config_path.name}')

        # Load data args
        data_config['record_path'] = str(bag_path)
        trial_data_config_path = write_temp_parameter_file(data_config)
        launch_arguments.append(f'record_config_path:={trial_data_config_path.name}')

        # Load trial args
        trial_config_path = write_temp_parameter_file(trial_config)
        launch_arguments.append(f'trial_config_path:={trial_config_path.name}')

        # Run The Trial
        launch_a_launch_file(
            launch_file_path=trial_launch,
            launch_file_arguments=launch_arguments,
        )

        # Save Metric Params
        params_path = bag_path / 'metric_params.yaml'
        yaml.safe_dump({'sim': sim_config, 'data': data_config, 'trial': trial_config}, open(params_path, 'w'))

        # Remove temp data
        trial_config_path.close()
        trial_data_config_path.close()
        trial_sim_config_path.close()


if __name__ == '__main__':
    main()
