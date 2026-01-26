from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='four_control',
            executable='visualization_node',
            name='visualization_node',
            output='screen'
        )

    ])
