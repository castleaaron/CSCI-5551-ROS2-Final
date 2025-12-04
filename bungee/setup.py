import os
from glob import glob
from setuptools import setup

package_name = 'bungee'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        ('share/bungee/launch', ['launch/bungee_control.launch.py']),
        (os.path.join('share', package_name), glob('resource/*rviz'))
        # (os.path.join('share', package_name), ['scripts/TerminatorScript.sh'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nick',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'offboard_control = bungee.offboard_control:main',
                'visualizer = bungee.visualizer:main',
                'velocity_control = bungee.velocity_control:main',
                'control = bungee.control:main',
                'xbox_in = bungee.xbox_in:main',
                'walking_setpoint = bungee.walking_setpoint:main',
                'processes = bungee.processes:main'
        ],
    },
)
