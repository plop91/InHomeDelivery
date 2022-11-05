from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera',
            namespace='camera',
            executable='img_publisher',
            name='sim'
        ),
        Node(
            package='motors',
            namespace='motors',
            executable='motors',
            name='sim'
        )
    ])