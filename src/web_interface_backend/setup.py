from setuptools import setup

package_name = 'web_interface_backend'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'flask', 'flask-socketio', 'opencv-python', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Backend for web interface',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'web_interface_node = web_interface_backend.web_interface_node:main',
            'visualization_server_node = web_interface_backend.visualization_server_node:main',
        ],
    },
)
