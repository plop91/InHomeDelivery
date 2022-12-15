"""
IMU node
Ian Sodersjerna
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion, Pose, Twist
from nav_msgs.msg import Odometry

import math
from .WT901 import WT901 as IMU


def quaternion_from_euler(ai: float, aj: float, ak: float):
    """
    Convert euler coordinate to quaternion.
    :param ai: i component of euler number
    :param aj: j component of euler number
    :param ak: k component of euler number
    :return:
    """
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
    """
    Imu node is responcible for interfacing with the imu and providing odometry information for a robot.
    """

    def __init__(self):
        """
        Initialize IMU node.
        """
        super().__init__('IMU_Node')

        # ROS parameters
        self.broadcast_interval = self.declare_parameter('broadcast_interval', 0.01)

        # IMU object
        self.imu = IMU()

        # ROS publishers
        self.odometry_publisher = self.create_publisher(Odometry, 'robot_odometry', 10)

        # ROS timer
        self.timer = self.create_timer(0.01, self.update)

    def update(self):
        """
        Update the imu and publish the odometry
        :return:
        """
        # update the IMU
        self.imu.update()

        # publish the robots odometry
        self.publish_odom()

    def publish_odom(self):
        """
        Publish the robots' odometry
        :return: None
        """
        # create the odometry object
        od = Odometry()

        # fill out the header and child frame
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = 'world'
        od.child_frame_id = 'None'

        # pose object - represents location
        p = Pose()

        # fill out the pose object
        p.position.x = self.imu.x_pos
        p.position.y = self.imu.y_pos
        p.position.z = 0.0

        p.orientation = quaternion_from_euler(0.0, 0.0, self.imu.z_pos)

        od.pose.pose = p

        # twist object - represents velocity
        t = Twist()

        # fill out twist object
        t.linear.x = self.imu.stcAcc.ax
        t.linear.y = self.imu.stcAcc.ay
        t.linear.z = self.imu.stcAcc.az

        t.angular.x = self.imu.stcGyro.gx
        t.angular.y = self.imu.stcGyro.gy
        t.angular.z = self.imu.stcGyro.gz

        od.twist.twist = t

        self.odometry_publisher.publish(od)


def main(args=None):
    """
    Main function of the program, initializes ROS.
    :param args: arguments passed to the program by ROS
    :return: exit code
    """
    rclpy.init(args=args)

    imu_node = IMUNode()

    rclpy.spin(imu_node)

    imu_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
