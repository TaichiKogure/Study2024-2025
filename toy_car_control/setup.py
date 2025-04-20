from setuptools import setup
import os
from glob import glob

package_name = 'toy_car_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='A simple ROS2 package for controlling a toy car with RaspberryPi',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = toy_car_control.publisher_node:main',
            'subscriber = toy_car_control.subscriber_node:main',
        ],
    },
)