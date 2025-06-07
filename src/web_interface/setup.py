from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'web_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web_app', 'src'), glob('web_app/src/*.py')),
        (os.path.join('share', package_name, 'web_app', 'src', 'templates'), glob('web_app/src/templates/*.html')),
        (os.path.join('share', package_name, 'web_app', 'src', 'static'), glob('web_app/src/static/*.*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Web interface for ROS2 perception and manipulation system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_interface_node = web_interface.web_interface_node:main',
        ],
    },
)
