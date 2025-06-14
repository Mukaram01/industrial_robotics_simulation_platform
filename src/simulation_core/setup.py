import os
from glob import glob
from setuptools import setup

package_name = 'simulation_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools', 'rclpy', 'pyyaml'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Simulation core logic',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'environment_configurator_node = simulation_core.environment_configurator_node:main',
            'safety_monitor_node = simulation_core.safety_monitor_node:main',
            'system_test_node = simulation_core.system_test_node:main',
        ],
    },
)
