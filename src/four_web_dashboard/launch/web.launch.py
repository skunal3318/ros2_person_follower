import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    http_port = LaunchConfiguration('http_port')

    declare_http_port = DeclareLaunchArgument(
        'http_port', default_value='8000',
        description='Port to serve the dashboard static files on.')

    rosbridge_launch = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml',
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch)
    )

    web_dir = os.path.join(
        get_package_share_directory('four_web_dashboard'), 'web')

    static_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', http_port],
        cwd=web_dir,
        output='screen',
    )

    return LaunchDescription([
        declare_http_port,
        rosbridge,
        static_server,
    ])
