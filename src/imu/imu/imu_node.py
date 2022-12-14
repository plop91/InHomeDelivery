import math
import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion, Pose, Twist
from nav_msgs.msg import Odometry

from .JY901 import JY901 as IMU


def quaternion_from_euler(ai: float, aj: float, ak: float):
    temp = Quaternion()

    temp.x = math.sin(ai / 2) * math.cos(aj / 2) * math.cos(ak / 2) - math.cos(ai / 2) * math.sin(
        aj / 2) * math.sin(ak / 2)
    temp.y = math.cos(ai / 2) * math.sin(aj / 2) * math.cos(ak / 2) + math.sin(ai / 2) * math.cos(
        aj / 2) * math.sin(ak / 2)
    temp.z = math.cos(ai / 2) * math.cos(aj / 2) * math.sin(ak / 2) - math.sin(ai / 2) * math.sin(
        aj / 2) * math.cos(ak / 2)
    temp.w = math.cos(ai / 2) * math.cos(aj / 2) * math.cos(ak / 2) + math.sin(ai / 2) * math.sin(
        aj / 2) * math.sin(ak / 2)

    return temp


class IMUNode(Node):

    def __init__(self):
        super().__init__('IMU_Node')
        # self.broadcast_interval = self.declare_parameter('broadcast_interval', 0.2).get_parameter_value().value

        self.imu = IMU()

        self.odometry_publisher = self.create_publisher(Odometry, 'robot_odometry', 10)

        self.timer = self.create_timer(0.01, self.update)

    def update(self):
        self.imu.update()
        self.publish_odom()

    def publish_odom(self):
        od = Odometry()

        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = 'world'
        od.child_frame_id = 'None'

        # pose represents location
        p = Pose()
        p.position.x = self.imu.x_pos
        p.position.y = self.imu.y_pos
        p.position.z = 0.0

        p.orientation = quaternion_from_euler(0.0, 0.0, self.imu.z_pos)

        od.pose.pose = p

        # twist represents velocity
        t = Twist()
        t.linear.x = self.imu.stcAcc.ax
        t.linear.y = self.imu.stcAcc.ay
        t.linear.z = self.imu.stcAcc.az

        t.angular.x = self.imu.stcGyro.gx
        t.angular.y = self.imu.stcGyro.gy
        t.angular.z = self.imu.stcGyro.gz

        od.twist.twist = t

        self.odometry_publisher.publish(od)


def main(args=None):
    rclpy.init(args=args)

    imu_node = IMUNode()

    rclpy.spin(imu_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
