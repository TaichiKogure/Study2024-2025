from setuptools import setup
import os
from glob import glob

package_name = 'turtle_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='A ROS2 package for simulating a turtle using Gazebo and RViz2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_keyboard = turtle_simulation.turtle_keyboard_node:main',
            'turtle_controller = turtle_simulation.turtle_controller_node:main',
            'gazebo_turtle = turtle_simulation.gazebo_turtle_node:main',
        ],
    },
)