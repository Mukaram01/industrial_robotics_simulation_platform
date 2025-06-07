from setuptools import find_packages
from setuptools import setup

setup(
    name='apm_msgs',
    version='0.1.0',
    packages=find_packages(
        include=('apm_msgs', 'apm_msgs.*')),
)
