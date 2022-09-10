import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mavros_params = os.path.join(
        get_package_share_directory('hawkeye_sim'),
        'config',
        'mavros_params.yaml',
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ign',
                'gazebo',
                '-r',
                'hawkeye_world.sdf',
            ],
        ),
        ExecuteProcess(
            cmd=[
                'sim_vehicle.py',
                '-w',
                '-v', 'ArduCopter',
                '-f', 'gazebo-iris',
                '--model', 'JSON',
            ],
        ),
        Node(
            package='mavros',
            executable='mavros_node',
            parameters=[mavros_params],
        ),
    ])
