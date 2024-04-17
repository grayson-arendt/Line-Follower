from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ir_pkg',
            executable='ir_pub',
            name='ir'
        ),
        Node(
            package='motor_pkg',
            executable='motor_sub',
            name='motor'
        ),
        Node(
            package='robot_pkg',
            executable='line_follower',
            name='line_follower',
            parameters=[
            {'speed': 60.0},
            {'direction': True}
            ]
        )
    ])