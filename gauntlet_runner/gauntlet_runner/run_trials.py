import argparse
from ros2launch.api import get_share_file_path_from_package, launch_a_launch_file
import tempfile
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('simulator_package')
    parser.add_argument('nav_config_package')
    args = parser.parse_args()

    trial_launch = get_share_file_path_from_package(
        package_name='gauntlet_runner',
        file_name='trial.launch.py')
    launch_arguments = []
    launch_arguments.append(f'simulator_package:={args.simulator_package}')
    launch_arguments.append(f'nav_config_package:={args.nav_config_package}')

    trial_config = {'goal_pose_x': 2.0}
    trial_config_path = tempfile.NamedTemporaryFile()
    yaml.safe_dump({'trial_runner': {'ros__parameters': trial_config}}, open(trial_config_path.name, 'w'))
    launch_arguments.append(f'trial_config_path:={trial_config_path.name}')

    launch_a_launch_file(
        launch_file_path=trial_launch,
        launch_file_arguments=launch_arguments,
    )

    trial_config_path.close()


if __name__ == '__main__':
    main()
