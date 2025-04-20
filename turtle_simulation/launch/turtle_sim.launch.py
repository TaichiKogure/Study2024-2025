from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
        ),
        
        # Launch keyboard controller node
        Node(
            package='turtle_simulation',
            executable='turtle_keyboard',
            name='turtle_keyboard_controller',
            output='screen',
            emulate_tty=True,
        ),
        
        # Launch turtle controller node
        Node(
            package='turtle_simulation',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen',
        ),
    ])