from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='four_control',
            executable='person_detector',
            name='person_detector',
            output='screen'
            parameters= [{'use_sim_time': True}]
        )

    ])
