from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'apm_core'

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
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include model files
        (os.path.join('share', package_name, 'models'), glob('models/*.onnx')),
        # Include class labels
        (os.path.join('share', package_name, 'config'), glob('config/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='user@example.com',
    description='Advanced Perception Module for ROS2 Humble',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_subscriber_node = apm_core.image_subscriber_node:main',
            'point_cloud_subscriber_node = apm_core.point_cloud_subscriber_node:main',
            'onnx_inference_node = apm_core.onnx_inference_node:main',
        ],
    },
)
