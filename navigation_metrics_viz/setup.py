from setuptools import setup

package_name = 'navigation_metrics_viz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David V. Lu!!',
    maintainer_email='davidvlu@gmail.com',
    description='Tools for visualizing the navigation_metrics',
    license='BSD',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['plot = navigation_metrics_viz.plot:main']},
)
