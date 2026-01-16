from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    side = LaunchConfiguration('side')
    gz_pose_topic = LaunchConfiguration('gz_pose_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'side',
            default_value='right',
            description='Side to stop next to the person (left/right)'
        ),
        DeclareLaunchArgument(
            'gz_pose_topic',
            default_value='/world/office/model/person/pose',
            description='Gazebo-bridged pose topic for the person model'
        ),

        Node(
            package='husky_person_follow',
            executable='person_detector_gz',
            name='person_detector_gz',
            output='screen',
            parameters=[{'gz_pose_topic': gz_pose_topic}],
        ),

        Node(
            package='husky_person_follow',
            executable='person_follow_fsm',
            name='person_follow_fsm',
            output='screen',
            parameters=[{'side': side}],
        ),
    ])
