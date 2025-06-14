from setuptools import setup

package_name = 'robot_interfaces'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
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
)
