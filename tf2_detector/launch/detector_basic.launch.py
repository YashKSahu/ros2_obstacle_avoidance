from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_detector',
            executable='tf_broadcaster',
            name='broadcaster',
            parameters=[{
                'use_sim_time': True
            }]
        ),
        Node(
            package='tf2_detector',
            executable='tf_listner',
            name='listner',
        )
    ])