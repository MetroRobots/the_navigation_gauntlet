from setuptools import setup

package_name = 'gauntlet_runner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/trial.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David V. Lu!!',
    maintainer_email='davidvlu@gmail.com',
    description='Utilities for running a series of simulated navigation tasks',
    license='BSD',
    entry_points={
        'console_scripts': [
            'run_trial = gauntlet_runner.run_trial:main',
            'run_trials = gauntlet_runner.run_trials:main',
        ]
    },
)
