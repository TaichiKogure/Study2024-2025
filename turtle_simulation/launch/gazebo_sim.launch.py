from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('turtle_simulation')
    
    # Gazebo world file
    world_file = os.path.join(pkg_share, 'worlds', 'turtle_world.world')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
        }.items(),
    )
    
    # Spawn turtle model
    spawn_turtle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtle',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen',
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(os.path.join(pkg_share, 'models', 'turtle.urdf'), 'r').read(),
            'use_sim_time': True,
        }],
    )
    
    # Launch keyboard controller node
    keyboard_controller = Node(
        package='turtle_simulation',
        executable='turtle_keyboard',
        name='turtle_keyboard_controller',
        output='screen',
        emulate_tty=True,
    )
    
    # Launch Gazebo turtle node
    gazebo_turtle = Node(
        package='turtle_simulation',
        executable='gazebo_turtle',
        name='gazebo_turtle',
        output='screen',
    )
    
    # Launch RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'turtle.rviz')],
        output='screen',
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_turtle,
        keyboard_controller,
        gazebo_turtle,
        rviz2,
    ])