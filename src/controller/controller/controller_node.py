import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .xbox_controller import XboxController


class Controller(Node):

    def __init__(self, update_interval=0.01):
        super().__init__('Controller_Node')

        # ROS parameters
        self.declare_parameter('dead_zone', 5)

        # Variables
        self.x = 0
        self.y = 0
        self.t = 0

        self.dead_zone = self.get_parameter('dead_zone').get_parameter_value().value

        # Controller object
        self.xbox_controller = XboxController()

        # ROS publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # ROS timer
        self.cmd_vel_pub_timer = self.create_timer(update_interval, self.update)

    def update(self):
        # self.xbox_controller.monitor_controller()
        self.read_controller()
        self.publish_cmd_vel()

    def read_controller(self):
        x, y, z = self.xbox_controller.read()

        tx = x * 100
        if self.dead_zone > tx > -self.dead_zone:
            self.x = 0.0
        else:
            self.x = tx

        ty = y * 100
        if self.dead_zone > ty > -self.dead_zone:
            self.y = 0.0
        else:
            self.y = ty

        tz = z * 100
        if self.dead_zone > tz > -self.dead_zone:
            self.t = 0.0
        else:
            self.t = tz

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.x
        msg.linear.y = self.y
        msg.angular.z = self.t
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing cmd_vel: x:{self.x:2} y:{self.y:2} z:{self.t:2}')


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
