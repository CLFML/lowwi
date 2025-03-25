from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lowwi',
            executable='lowwi_node',
            name='lowwi_node',
            output='screen',
            parameters=['./example/LOWWI_demo_ros2/params.yaml']  # Path to your params file
        )
    ])
