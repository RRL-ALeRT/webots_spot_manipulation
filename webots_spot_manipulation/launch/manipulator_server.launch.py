import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('webots_spot_manipulation'), 'config', 'params.yaml')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_test_frame', default_value='true',
                              description='Publish the static test frame for /move_to_frame'),
        DeclareLaunchArgument('test_frame', default_value='man_test'),
        DeclareLaunchArgument('test_parent_frame', default_value='kinova_color'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        Node(
            package='webots_spot_manipulation',
            executable='manipulator_server',
            name='manipulator_action_server',
            output='screen',
            parameters=[params, use_sim_time],
        ),
        Node(
            package='webots_spot_manipulation',
            executable='gripper_server',
            name='gripper_server',
            output='screen',
            parameters=[params, use_sim_time],
        ),
        # Static test frame "man_test" for /move_to_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='man_test_frame_publisher',
            output='screen',
            arguments=[
                '--x', LaunchConfiguration('x'),
                '--y', LaunchConfiguration('y'),
                '--z', LaunchConfiguration('z'),
                '--roll', LaunchConfiguration('roll'),
                '--pitch', LaunchConfiguration('pitch'),
                '--yaw', LaunchConfiguration('yaw'),
                '--frame-id', LaunchConfiguration('test_parent_frame'),
                '--child-frame-id', LaunchConfiguration('test_frame'),
            ],
            parameters=[use_sim_time],
            condition=IfCondition(LaunchConfiguration('publish_test_frame')),
        ),
    ])
