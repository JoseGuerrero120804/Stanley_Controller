import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('stanley_controller'),
        'config',
        'actual_trajectory_recorder_params.yaml',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to path_recorder parameter YAML file.',
    )

    path_recorder_node = Node(
        package='stanley_controller',
        executable='path_recorder',
        name='path_recorder',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        path_recorder_node,
    ])
