from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='toy_car_control',
            executable='publisher',
            name='toy_car_publisher',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='toy_car_control',
            executable='subscriber',
            name='toy_car_controller',
            output='screen',
        ),
    ])