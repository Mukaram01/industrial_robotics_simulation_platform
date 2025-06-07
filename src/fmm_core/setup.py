from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'fmm_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='user@example.com',
    description='Flexible Manipulation Module for ROS2 Humble',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planning_scene_updater_node = fmm_core.planning_scene_updater_node:main',
            'fmm_moveit_interface_node = fmm_core.fmm_moveit_interface_node:main',
        ],
    },
)
