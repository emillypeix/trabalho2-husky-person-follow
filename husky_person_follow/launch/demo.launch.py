from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    side = LaunchConfiguration('side')

    return LaunchDescription([
        DeclareLaunchArgument(
            'side',
            default_value='right',
            description='Side to stop next to the person (left/right)'
        ),

        Node(
            package='husky_person_follow',
            executable='person_detector',
            name='person_detector',
            output='screen',
        ),

        Node(
            package='husky_person_follow',
            executable='person_follow_fsm',
            name='person_follow_fsm',
            output='screen',
            parameters=[{'side': side}],
        ),
    ])
