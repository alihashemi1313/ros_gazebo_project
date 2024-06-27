from setuptools import setup

package_name = 'pidcontrollerpkg'
submodules = 'pidcontrollerpkg/sevenseg'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='narges',
    maintainer_email='nargesrajabiun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'pidnode1 = pidcontrollerpkg.set_Tourqe_AN:main',
        	'pidnode2 = pidcontrollerpkg.PidOptAll_V6:main',
        	'pidnode3 = pidcontrollerpkg.PID_controller_2:main',
        	'pidinv = pidcontrollerpkg.PID_controller_Inv:main',
            'settorque = pidcontrollerpkg.set_Tourqe:main',

        ],
    },
)
