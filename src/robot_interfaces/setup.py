from setuptools import setup, find_packages

package_name = 'robot_interfaces'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Robot specific interfaces',
    license='Apache-2.0',
    entry_points={
        'simulation_core.robots': [
            'delta = robot_interfaces.delta_interface:DeltaInterface',
            'ur5 = robot_interfaces.ur5_interface:UR5Interface',
        ],
    },
)
