import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

from .xbox_controller import XboxController


class Controller(Node):

    def __init__(self, update_interval=0.2):
        super().__init__('Controller')

        self.x = 0
        self.y = 0
        self.t = 0

        self.xbox_controller = XboxController()
        self.xbox_timer = self.create_timer(0.1, self.xbox_controller.monitor_controller())

        self.X_publisher_ = self.create_publisher(String, 'CMD_X', 10)
        self.Y_publisher_ = self.create_publisher(String, 'CMD_Y', 10)
        self.T_publisher_ = self.create_publisher(String, 'CMD_T', 10)
        self.cmd_vel_pub_timer = self.create_timer(update_interval, self.publish_cmd_vel)

    def read_controller(self):
        x, y, z = self.xbox_controller.read()
        self.x = int(x * 10)
        self.y = int(y * 10)
        self.t = int(z * 10)

    def publish_cmd_vel(self):
        msg = String()
        msg.data = "x=" + str(self.x)
        self.X_publisher_.publish(msg)
        self.get_logger().info('Publishing x: "%s"' % msg.data)
        msg = String()
        msg.data = "y=" + str(self.y)
        self.Y_publisher_.publish(msg)
        self.get_logger().info('Publishing y: "%s"' % msg.data)
        msg = String()
        msg.data = "t=" + str(self.t)
        self.T_publisher_.publish(msg)
        self.get_logger().info('Publishing t: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
