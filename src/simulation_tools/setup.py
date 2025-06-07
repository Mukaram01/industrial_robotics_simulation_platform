import os
from glob import glob
from setuptools import setup

package_name = 'simulation_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'templates'), glob(os.path.join('templates', '*.html'))),
        (os.path.join('share', package_name, 'static'), glob(os.path.join('static', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Industrial Robotics Simulation Tools',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_simulator_node = simulation_tools.camera_simulator_node:main',
            'web_interface_node = simulation_tools.web_interface_node:main',
            'environment_configurator_node = simulation_tools.environment_configurator_node:main',
            'visualization_server_node = simulation_tools.visualization_server_node:main',
            'industrial_protocol_bridge_node = simulation_tools.industrial_protocol_bridge_node:main',
            'safety_monitor_node = simulation_tools.safety_monitor_node:main',
            'system_test_node = simulation_tools.system_test_node:main',
        ],
    },
)
