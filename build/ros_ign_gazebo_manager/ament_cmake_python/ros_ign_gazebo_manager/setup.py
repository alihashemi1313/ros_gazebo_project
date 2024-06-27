from setuptools import find_packages
from setuptools import setup

setup(
    name='ros_ign_gazebo_manager',
    version='0.0.9',
    packages=find_packages(
        include=('ros_ign_gazebo_manager', 'ros_ign_gazebo_manager.*')),
)
