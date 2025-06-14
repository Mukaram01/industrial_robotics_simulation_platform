from setuptools import setup

package_name = 'perception_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'numpy', 'opencv-python', 'cv_bridge'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Perception nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'synthetic_camera_node = perception_nodes.synthetic_camera_node:main',
        ],
    },
)
