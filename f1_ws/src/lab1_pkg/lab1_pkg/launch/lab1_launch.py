import f1_ws.src.lab1_pkg.lab1_pkg.launch.launch as launch
from f1_ws.src.lab1_pkg.lab1_pkg.launch.launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare parameter for 'talker' node
        DeclareLaunchArgument(
            'v',
            default_value='2.0',
            description='Speed parameter for talker node'),
        DeclareLaunchArgument(
            'd',
            default_value='-0.5',
            description='Steering angle parameter for talker node'),

        # Launch the 'talker' node
        Node(
            package='lab1_pkg',
            executable='talker',
            name='talker',
            output='screen',
            parameters=[{'v': LaunchConfiguration('v'), 'd': LaunchConfiguration('d')}]
        ),
        
        # Launch the 'listener' node (assuming it is implemented in your package)
        Node(
            package='lab1_pkg',
            executable='relay',
            name='relay',
            output='screen'
        ),
    ])
