import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import lgpio
from .motor import Motor


class Motors(Node):

    def __init__(self):
        super().__init__('Robot')

        # ROS parameters
        self.declare_parameter('update_interval', 0.2)

        # variables
        self.front_left_wheel_speed = 0
        self.front_right_wheel_speed = 0
        self.back_left_wheel_speed = 0
        self.back_right_wheel_speed = 0

        self.command_vel_x = 0
        self.command_vel_y = 0
        self.command_vel_t = 0

        # initialize GPIO
        self.gpio = lgpio.gpiochip_open(0)

        self.front_left_motor = Motor(self.gpio, 18, 24, 17, 25, 5)
        self.front_right_motor = Motor(self.gpio, 6, 16, 19, 20, 12)
        self.back_left_motor = Motor(self.gpio, 23, 22, 4, 21, 27)
        self.back_right_motor = Motor(self.gpio, 13, 26, 11, 9, 10)

        # Subscribe to CMD velocities
        # TODO: Make CMD velocities not strings
        self.X_subscription = self.create_subscription(
            String,
            'CMD_X',
            self.x_listener_callback,
            10)
        self.Y_subscription = self.create_subscription(
            String,
            'CMD_Y',
            self.y_listener_callback,
            10)
        self.T_subscription = self.create_subscription(
            String,
            'CMD_T',
            self.t_listener_callback,
            10)
        # TODO: Figure out if I really need this
        self.X_subscription  # prevent unused variable warning
        self.Y_subscription  # prevent unused variable warning
        self.T_subscription  # prevent unused variable warning

        # Publish odom information
        self.fl_odom_publisher_ = self.create_publisher(String, 'fl_odom', 10)
        self.fr_odom_publisher_ = self.create_publisher(String, 'fr_odom', 10)
        self.bl_odom_publisher_ = self.create_publisher(String, 'bl_odom', 10)
        self.br_odom_publisher_ = self.create_publisher(String, 'br_odom', 10)

        update_interval = self.get_parameter('update_interval').value
        self.odom_timer = self.create_timer(update_interval, self.update_odometry)

        self.update_timer = self.create_timer(update_interval, self.update)

    def __del__(self):
        lgpio.gpiochip_close(self.gpio)

    def x_listener_callback(self, msg):
        self.command_vel_x = int(msg.data)
        self.get_logger().info('X heard: "%s"' % msg.data)

    def y_listener_callback(self, msg):
        self.command_vel_y = int(msg.data)
        self.get_logger().info('Y heard: "%s"' % msg.data)

    def t_listener_callback(self, msg):
        self.command_vel_t = int(msg.data)
        self.get_logger().info('T heard: "%s"' % msg.data)

    def update(self):
        self.get_logger().info('updating motors')
        self.mix_inputs()
        self.set_motor_speed()

    def update_odometry(self):
        msg = String()
        msg.data = str(self.front_left_motor.pos)
        self.fl_odom_publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg.data))

        msg = String()
        msg.data = str(self.front_right_motor.pos)
        self.fr_odom_publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg.data))

        msg = String()
        msg.data = str(self.back_left_motor.pos)
        self.bl_odom_publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg.data))

        msg = String()
        msg.data = str(self.back_right_motor.pos)
        self.br_odom_publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg.data))

    def mix_inputs(self):
        sig = 0
        front_left_wheel_speed = 0
        front_right_wheel_speed = 0
        back_left_wheel_speed = 0
        back_right_wheel_speed = 0

        if self.command_vel_x > 0:
            sig += 1
            front_left_wheel_speed += self.command_vel_x
            front_right_wheel_speed += self.command_vel_x
            back_left_wheel_speed += self.command_vel_x
            back_right_wheel_speed += self.command_vel_x
        elif self.command_vel_x < 0:
            sig += 1
            front_left_wheel_speed += self.command_vel_x
            front_right_wheel_speed += self.command_vel_x
            back_left_wheel_speed += self.command_vel_x
            back_right_wheel_speed += self.command_vel_x

        if self.command_vel_y > 0:
            sig += 1
            front_left_wheel_speed += self.command_vel_x
            front_right_wheel_speed -= self.command_vel_x
            back_left_wheel_speed -= self.command_vel_x
            back_right_wheel_speed += self.command_vel_x
        elif self.command_vel_y < 0:
            sig += 1
            front_left_wheel_speed -= self.command_vel_x
            front_right_wheel_speed += self.command_vel_x
            back_left_wheel_speed += self.command_vel_x
            back_right_wheel_speed -= self.command_vel_x

        if self.command_vel_t > 0:
            sig += 1
            front_left_wheel_speed -= self.command_vel_x
            front_right_wheel_speed += self.command_vel_x
            back_left_wheel_speed -= self.command_vel_x
            back_right_wheel_speed += self.command_vel_x
        elif self.command_vel_t < 0:
            sig += 1
            front_left_wheel_speed += self.command_vel_x
            front_right_wheel_speed -= self.command_vel_x
            back_left_wheel_speed += self.command_vel_x
            back_right_wheel_speed -= self.command_vel_x

        if sig == 0:
            self.front_left_wheel_speed = 0
            self.front_right_wheel_speed = 0
            self.back_left_wheel_speed = 0
            self.back_right_wheel_speed = 0
            return

        front_left_wheel_speed = front_left_wheel_speed / sig
        front_right_wheel_speed = front_right_wheel_speed / sig
        back_left_wheel_speed = back_left_wheel_speed / sig
        back_right_wheel_speed = back_right_wheel_speed / sig

        self.front_left_wheel_speed = front_left_wheel_speed
        self.front_right_wheel_speed = front_right_wheel_speed
        self.back_left_wheel_speed = back_left_wheel_speed
        self.back_right_wheel_speed = back_right_wheel_speed

    def set_motor_speed(self):
        self.front_left_motor.set_speed(self.front_left_wheel_speed)
        self.front_right_motor.set_speed(self.front_right_wheel_speed)
        self.back_left_motor.set_speed(self.back_left_wheel_speed)
        self.back_right_motor.set_speed(self.back_right_wheel_speed)


def main(args=None):
    rclpy.init(args=args)

    motors = Motors()

    rclpy.spin(motors)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    motors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
