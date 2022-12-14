from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera',
            executable='img_pub',
            name='Delivery'
        ),
        Node(
            package='camera',
            executable='marker_pose_pub',
            name='Delivery'
        ),
        # Node(
        #     package='imu',
        #     namespace='imu',
        #     executable='imu',
        #     name='Delivery'
        # ),
        # Node(
        #     package='motors',
        #     namespace='motors',
        #     executable='motors',
        #     name='Delivery'
        # )
    ])
