from setuptools import setup

package_name = 'industrial_protocols'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'asyncua', 'paho-mqtt', 'pyyaml', 'pymodbus'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Industrial communication protocols',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'industrial_protocol_bridge_node = industrial_protocols.industrial_protocol_bridge_node:main',
        ],
    },
)
