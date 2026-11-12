import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('rov_motor_simulator'),
        'config', 'motor_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with the motor_simulator parameters'),
        Node(
            package='rov_motor_simulator',
            executable='motor_simulator_node',
            name='motor_simulator',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'),
    ])
