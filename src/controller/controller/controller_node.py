"""
Controller Node
Ian Sodersjerna
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .xbox_controller import XboxController


class Controller(Node):
    """
    Controller Node, this node publishes the state of the controller as a twist, this can be easily expanded to
    include additional published topics for buttons or combinations of inputs.
    """

    def __init__(self, update_interval=0.01):
        """
        Initialize the controller node
        :param update_interval: How ofter will controller update be called.
        """
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
        """
        Update the controller node
        :return: None
        """
        # update controller values.
        self.read_controller()

        # publish controller values as command velocity
        self.publish_cmd_vel()

    def read_controller(self):
        """
        Read the controller and check if the values are outside the dead-zone.
        :return: None
        """
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
        """
        Publish command velocities as a twist.
        :return: None
        """
        # create twist object
        msg = Twist()

        # assign values.
        msg.linear.x = self.x
        msg.linear.y = self.y
        msg.angular.z = self.t

        # publish command velocities
        self.publisher.publish(msg)

        # log command velocities
        self.get_logger().info(f'Publishing cmd_vel: x:{self.x:2} y:{self.y:2} z:{self.t:2}')


def main(args=None):
    """
    Main function of the program, initializes ROS.
    :param args: arguments passed to the program by ROS
    :return: exit code
    """
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    controller.destroy_node()

    rclpy.shutdown()

    return 0


if __name__ == '__main__':
    exit(main())
