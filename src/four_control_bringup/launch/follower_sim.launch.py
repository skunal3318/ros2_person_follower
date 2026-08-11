import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_web = LaunchConfiguration('launch_web')

    declare_launch_web = DeclareLaunchArgument(
        'launch_web', default_value='true',
        description='Also start rosbridge + the browser dashboard.')

    rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('four_control_bringup'),
                'launch',
                'rover.launch.py',
            )
        )
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('four_control'),
                'launch',
                'perception.launch.py',
            )
        ),
        launch_arguments={
            # Gazebo already publishes /camera/image_raw via ros_gz_bridge,
            # and gz-sim-diff-drive-system already publishes /odom + tf.
            'use_camera_node': 'false',
            'publish_odom': 'false',
            'use_sim_time': 'true',
        }.items(),
    )

    web = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('four_web_dashboard'),
                'launch',
                'web.launch.py',
            )
        ),
        condition=IfCondition(launch_web),
    )

    return LaunchDescription([
        declare_launch_web,
        rover,
        perception,
        web,
    ])
