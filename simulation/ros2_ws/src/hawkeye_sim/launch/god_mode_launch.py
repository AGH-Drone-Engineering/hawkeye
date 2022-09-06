from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ExecuteProcess(
        #     cmd=[
        #         'ign',
        #         'gazebo',
        #         '-r',
        #         'hawkeye_world.sdf',
        #     ],
        # ),
        # ExecuteProcess(
        #     cmd=[
        #         'sim_vehicle.py',
        #         '-w',
        #         '-v', 'ArduCopter',
        #         '-f', 'gazebo-iris',
        #         '--model', 'JSON',
        #         '-m', '--out=tcpin:0.0.0.0:8888',
        #     ],
        # ),
        # Node(
        #     package='mavros',
        #     executable='mavros_node',
        #     parameters=[{
        #         'fcu_url': 'tcp://localhost:8888'
        #     }],
        # ),
        Node(
            package='hawkeye_control',
            executable='all_vehicle_types',
            remappings=[
                ('target/vec', 'iris/iris_dummy/vec'),
            ],
        ),
        Node(
            package='hawkeye_sim',
            executable='gods_hand',
        ),
        Node(
            package='hawkeye_sim',
            executable='logical_camera',
            remappings=[
                ('target/vec', 'iris/iris_dummy/vec'),
            ],
        ),
    ])
