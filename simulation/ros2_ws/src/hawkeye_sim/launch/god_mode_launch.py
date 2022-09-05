from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ExecuteProcess(cmd=['ign', 'gazebo', '-r', 'hawkeye_world.sdf']),
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
            remappings=[
                ('pose', 'iris_dummy/pose'),
            ],
        ),
        Node(
            package='hawkeye_sim',
            executable='gods_eye',
            remappings=[
                ('follower/pose', '/mavros/local_position/pose'),
                ('target/pose', 'iris_dummy/pose'),
                ('follower/target/vec', 'iris/iris_dummy/vec'),
            ],
        ),
    ])
