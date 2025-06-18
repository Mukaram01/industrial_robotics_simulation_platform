import os
from glob import glob
from setuptools import setup

package_name = 'web_interface_frontend'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'templates'), glob('templates/*.html')),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Frontend resources for web interface',
    license='Apache-2.0',
)
