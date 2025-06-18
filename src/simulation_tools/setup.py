import os
from glob import glob
from setuptools import setup

package_name = 'simulation_tools'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Industrial Robotics Simulation Tools',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'object_3d_viewer = simulation_tools.object_3d_viewer_node:main',
        ]
    },
)
