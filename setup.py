from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'stanley_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nombre_usuario',
    maintainer_email='a01736556@tec.mx',
    description='Stanley lateral controller + PID longitudinal speed controller '
                'for following a recorded path',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_recorder = stanley_controller.path_recorder:main',
            'stanley_controller = stanley_controller.stanley_node:main',
            'evaluate_tracking = stanley_controller.evaluate_tracking:main',
            'qcar_teleop_bridge = stanley_controller.qcar_teleop_bridge:main',
            'mocap_to_waypoints = stanley_controller.mocap_to_waypoints:main',
            'dead_reckoning = stanley_controller.dead_reckoning:main',
        ],
    },
)
