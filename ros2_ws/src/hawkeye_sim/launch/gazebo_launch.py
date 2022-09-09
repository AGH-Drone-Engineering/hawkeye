from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
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
            parameters=[{
                'fcu_url': 'udp://127.0.0.1:14550@',
                'plugin_allowlist': [
                    'sys_*',
                    'command',
                    'global_position',
                    'local_position',
                    'setpoint_velocity',
                ],
            }],
        ),
    ])
