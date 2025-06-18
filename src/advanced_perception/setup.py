from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'advanced_perception'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'numpy',
        'python3-opencv',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Advanced perception package for object segmentation and pose estimation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'segmentation_node = advanced_perception.segmentation_node:main',
            'pose_estimation_node = advanced_perception.pose_estimation_node:main',
        ],
        'simulation_core.perception_nodes': [
            'segmentation = advanced_perception.segmentation_node:SegmentationNode',
            'pose_estimation = advanced_perception.pose_estimation_node:PoseEstimationNode',
        ],
    },
)
