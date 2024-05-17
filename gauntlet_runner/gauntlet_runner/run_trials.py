import argparse
from ament_index_python.packages import get_package_share_path, PackageNotFoundError
from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnShutdown
from launch.substitutions import LocalSubstitution
from ros2launch.api.api import get_share_file_path_from_package, parse_launch_arguments
from .exploration import explore_parameter_space, format_value
from resource_retriever import get_filename
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


def write_temp_parameter_file(parameters, node_name='/**', ros_params=True):
    temp_config_path = tempfile.NamedTemporaryFile()
    if ros_params:
        yaml_data = {node_name: {'ros__parameters': parameters}}
    else:
        yaml_data = parameters
    yaml.safe_dump(yaml_data, open(temp_config_path.name, 'w'))
    return temp_config_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('trials_config_path', type=pathlib.Path)
    parser.add_argument('-f', '--force', action='store_true', help='overwrite existing data')
    parser.add_argument('-g', '--gui', action='store_true', help='show the graphics')
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

        robot_name = sim_config['robot_name']
        launch_arguments.append(f'robot_name:={robot_name}')

        trial_sim_config_path = write_temp_parameter_file(sim_config, ros_params=False)
        launch_arguments.append(f'sim_config_path:={trial_sim_config_path.name}')

        additional_data = []
        for config in [sim_pkg_config]:
            data_config['topics'] += config.get('topics', [])
            additional_data += config.get('additional_data', [])

        # Load data args
        data_config['record_path'] = str(bag_path)
        trial_data_config_path = write_temp_parameter_file(data_config)
        launch_arguments.append(f'record_config_path:={trial_data_config_path.name}')

        if args.gui:
            launch_arguments.append('gui:=True')

        # Load trial args
        trial_config_path = write_temp_parameter_file(trial_config)
        launch_arguments.append(f'trial_config_path:={trial_config_path.name}')

        exited = False

        # Run The Trial
        def shutdown_callback(context, reason_sub):
            nonlocal exited
            reason = reason_sub.perform(context)
            if 'SIGINT' in str(reason):
                exited = True
            for i in range(10):
                print(reason)

        launch_service = LaunchService()
        launch_description = LaunchDescription([
            IncludeLaunchDescription(
                trial_launch,
                launch_arguments=parse_launch_arguments(launch_arguments),
            ),
            RegisterEventHandler(OnShutdown(
                on_shutdown=[
                    OpaqueFunction(
                        function=shutdown_callback,
                        args=[LocalSubstitution('event.reason')],
                    ),
                ]
            ))
        ])
        launch_service.include_launch_description(launch_description)
        launch_service.run()

        bag_path.mkdir(exist_ok=True, parents=True)

        # Save additional data
        for data_path in additional_data:
            data_f = get_filename(data_path, False)
            # https://stackoverflow.com/a/51108375
            p = pathlib.Path(data_f).expanduser()
            parts = p.parts[p.is_absolute():]
            for resolved in pathlib.Path(p.root).glob(str(pathlib.Path(*parts))):
                resolved.rename(bag_path / resolved.name)

        # Save Metric Params
        params_path = bag_path / 'metric_params.yaml'
        metric_params = {}
        for ns, config in {'sim': sim_config, 'data': data_config, 'trial': trial_config}.items():
            for k, v in config.items():
                metric_params[f'{ns}/{k}'] = v
        yaml.safe_dump(metric_params, open(params_path, 'w'))

        # Remove temp data
        trial_config_path.close()
        trial_data_config_path.close()
        trial_sim_config_path.close()

        if exited:
            break


if __name__ == '__main__':
    main()
