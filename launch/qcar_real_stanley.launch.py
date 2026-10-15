import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _nodes(context):
    pose_source = context.launch_configurations['pose_source']
    if pose_source not in ('mocap', 'dead_reckoning'):
        raise RuntimeError("pose_source must be 'mocap' or 'dead_reckoning'")
    params_file = context.launch_configurations['params_file'] or os.path.join(
        get_package_share_directory('stanley_controller'), 'config',
        f'qcar_real_{pose_source}_params.yaml')

    nodes = [Node(
        package='stanley_controller',
        executable='stanley_controller',
        name='stanley_controller',
        output='screen',
        parameters=[params_file],
    )]
    if pose_source == 'dead_reckoning':
        nodes.append(Node(
            package='stanley_controller',
            executable='dead_reckoning',
            name='dead_reckoning',
            output='screen',
            parameters=[params_file],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pose_source',
            default_value='mocap',
            description="'mocap' (live OptiTrack PoseStamped) or 'dead_reckoning' "
                        '(tachometer + steering, no cameras).',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Parameter YAML (default: config/qcar_real_<pose_source>_params.yaml).',
        ),
        OpaqueFunction(function=_nodes),
    ])
