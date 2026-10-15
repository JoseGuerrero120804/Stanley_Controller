import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('stanley_controller'),
        'config',
        'qcar_stanley_params.yaml',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to stanley_controller parameter YAML file.',
    )

    launch_sim_arg = DeclareLaunchArgument(
        'launch_sim',
        default_value='true',
        description='Also start the QCar Gazebo simulation (qcar_gazebo).',
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='test_world.sdf',
        description='qcar_gazebo world file (must match the one used to record the path).',
    )

    sim_startup_delay_arg = DeclareLaunchArgument(
        'sim_startup_delay',
        default_value='10.0',
        description='Seconds to wait for Gazebo, ros2_control and the vehicle_controller '
                    'to come up before starting the controller.',
    )

    qcar_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('qcar_gazebo'), 'launch'),
            '/gz_sim.launch.py',
        ]),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
        condition=IfCondition(LaunchConfiguration('launch_sim')),
    )

    stanley_node = Node(
        package='stanley_controller',
        executable='stanley_controller',
        name='stanley_controller',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    delayed_stanley_node = TimerAction(
        period=LaunchConfiguration('sim_startup_delay'),
        actions=[stanley_node],
    )

    return LaunchDescription([
        params_file_arg,
        launch_sim_arg,
        world_arg,
        sim_startup_delay_arg,
        qcar_sim,
        delayed_stanley_node,
    ])
