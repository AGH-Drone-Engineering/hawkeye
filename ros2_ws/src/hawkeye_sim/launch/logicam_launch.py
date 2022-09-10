from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='hawkeye_sim',
        #     executable='gods_hand',
        # ),
        Node(
            package='hawkeye_sim',
            executable='logical_camera',
        ),
        Node(
            package='hawkeye_sim',
            executable='tf2vec',
            parameters=[{
                'target_frame': 'iris_dummy',
            }],
            remappings=[
                ('vec', 'drone/target/vec'),
            ],
        ),
        Node(
            package='hawkeye_control',
            executable='all_vehicle_types',
            remappings=[
                ('target/vec', 'drone/target/vec'),
            ],
        ),
    ])
