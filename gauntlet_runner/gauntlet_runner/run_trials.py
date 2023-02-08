import argparse
from ros2launch.api import get_share_file_path_from_package, launch_a_launch_file


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

    launch_a_launch_file(
        launch_file_path=trial_launch,
        launch_file_arguments=launch_arguments,
    )


if __name__ == '__main__':
    main()
