from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_camera_node = LaunchConfiguration('use_camera_node')
    publish_odom = LaunchConfiguration('publish_odom')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_camera_node = DeclareLaunchArgument(
        'use_camera_node', default_value='true',
        description='Launch camera_node to capture a real webcam. '
                     'Set false when a simulator already publishes /camera/image_raw.')
    declare_publish_odom = DeclareLaunchArgument(
        'publish_odom', default_value='true',
        description='Have controller_node integrate and publish its own /odom + tf. '
                     'Set false when a simulator/robot already provides odometry.')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    sim_time_param = {'use_sim_time': use_sim_time}

    camera = Node(
        package='four_control',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[sim_time_param],
        condition=IfCondition(use_camera_node),
    )

    detector = Node(
        package='four_control',
        executable='person_detector_node',
        name='person_detector_node',
        output='screen',
        parameters=[sim_time_param],
    )

    controller = Node(
        package='four_control',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[sim_time_param, {'publish_odom': publish_odom}],
    )

    viewer = Node(
        package='four_control',
        executable='viewer_node',
        name='viewer_node',
        output='screen',
        parameters=[sim_time_param],
    )

    return LaunchDescription([
        declare_use_camera_node,
        declare_publish_odom,
        declare_use_sim_time,
        camera,
        detector,
        controller,
        viewer,
    ])
