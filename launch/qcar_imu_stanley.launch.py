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
        'qcar_imu_params.yaml',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Parameter YAML for imu_odometry and stanley_controller.',
    )

    imu_odometry_node = Node(
        package='stanley_controller',
        executable='dead_reckoning',
        name='imu_odometry',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    stanley_node = Node(
        package='stanley_controller',
        executable='stanley_controller',
        name='stanley_controller',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        imu_odometry_node,
        stanley_node,
    ])
