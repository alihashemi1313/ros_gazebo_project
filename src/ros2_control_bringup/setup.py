from setuptools import setup
import os
from glob import glob 

package_name = 'ros2_control_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
        (os.path.join('share', package_name,'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name,'config'), glob('config/*')),
        (os.path.join('share', package_name,'world'), glob('world/*')),
        (os.path.join('share', package_name,'sdf'), glob('sdf/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nargas',
    maintainer_email='nargas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'CtrlTest = ros2_control_bringup.controller_test:main',
        	'CtrlInvTest = ros2_control_bringup.controller_Inv_test:main',
        	'ball = ros2_control_bringup.ball_spawn:main',
        ],
    },
)


