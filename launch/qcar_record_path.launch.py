import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('stanley_controller'),
        'config',
        'qcar_path_recorder_params.yaml',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to path_recorder parameter YAML file.',
    )

    keyboard_bridge_arg = DeclareLaunchArgument(
        'keyboard_bridge',
        default_value='true',
        description='Start qcar_teleop_bridge (teleop_twist_keyboard /cmd_vel -> '
                    '/qcar_sim/user_command). Set false when driving with the joystick.',
    )

    path_recorder_node = Node(
        package='stanley_controller',
        executable='path_recorder',
        name='path_recorder',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    teleop_bridge_node = Node(
        package='stanley_controller',
        executable='qcar_teleop_bridge',
        name='qcar_teleop_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('keyboard_bridge')),
    )

    return LaunchDescription([
        params_file_arg,
        keyboard_bridge_arg,
        path_recorder_node,
        teleop_bridge_node,
    ])
