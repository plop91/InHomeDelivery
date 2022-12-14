import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from delivery_robot_interfaces.srv import MotorMovement

import lgpio
from .motor import Motor


class Motors(Node):

    def __init__(self):
        super().__init__('Motors_Node')

        # ROS parameters
        self.declare_parameter('update_interval', 0.01)
        self.declare_parameter('PWM_FREQ', 500)
        self.declare_parameter('command_type', 'speed')
        self.declare_parameter('mix_type', 'mix')

        # variables
        self.pwm_freq = float(self.get_parameter('PWM_FREQ').value)

        self.front_left_wheel_speed = 0
        self.front_right_wheel_speed = 0
        self.back_left_wheel_speed = 0
        self.back_right_wheel_speed = 0

        self.last_front_left_wheel_speed = 0
        self.last_front_right_wheel_speed = 0
        self.last_back_left_wheel_speed = 0
        self.last_back_right_wheel_speed = 0

        self.front_left_wheel_pos = 0
        self.front_right_wheel_pos = 0
        self.back_left_wheel_pos = 0
        self.back_right_wheel_pos = 0

        self.last_front_left_wheel_pos = 0
        self.last_front_right_wheel_pos = 0
        self.last_back_left_wheel_pos = 0
        self.last_back_right_wheel_pos = 0

        self.front_left_wheel_posi = 0
        self.front_right_wheel_posi = 0
        self.back_left_wheel_posi = 0
        self.back_right_wheel_posi = 0

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
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.update_interval = float(self.get_parameter('update_interval').value)
        self.update_timer = self.create_timer(self.update_interval, self.update)

        self.motors_enabled = True
        self.disable_srv = self.create_service(Empty, 'disable_motors', self.disable_motors)
        self.enable_srv = self.create_service(Empty, 'enable_motors', self.enable_motors)

        self.forward_srv = self.create_service(MotorMovement, 'drive_forward', self.drive_forward)
        self.backward_srv = self.create_service(MotorMovement, 'drive_backward', self.drive_backward)

        self.left_srv = self.create_service(MotorMovement, 'drive_left', self.drive_left)
        self.right_srv = self.create_service(MotorMovement, 'drive_right', self.drive_right)

        self.cw_srv = self.create_service(MotorMovement, 'turn_cw', self.turn_cw)
        self.ccw_srv = self.create_service(MotorMovement, 'turn_ccw', self.turn_ccw)

        self.motors_calibrated = False
        self.calibration_srv = self.create_service(Empty, 'calibrate_motors', self.calibrate_motors)

    def cmd_vel_callback(self, msg):
        self.command_vel_x = msg.linear.y
        self.command_vel_y = msg.linear.x
        self.command_vel_t = msg.angular.z
        self.get_logger().info(f'cmd_vel received: x:{self.command_vel_x:2} '
                               f'y:{self.command_vel_y:2} z:{self.command_vel_t:2}')

    def update(self):
        # input type
        mix_type = self.get_parameter('mix_type').get_parameter_value().string_value
        if mix_type == "mix":
            self.mix_inputs()
        elif mix_type == "max":
            self.max_inputs()
        else:
            print("invalid mix type")

        pwm_freq = float(self.get_parameter('PWM_FREQ').value)
        if pwm_freq != self.pwm_freq:
            self.get_logger().info(f"pwm freq updated:{pwm_freq}")
            self.pwm_freq = pwm_freq
            self.set_pwm_freq(self.pwm_freq)

        # output type
        command_type = self.get_parameter('command_type').get_parameter_value().string_value
        if command_type == "speed":
            self.set_motor_speed()
        elif command_type == "vel":
            self.set_motor_velocity()
        elif command_type == "pos":
            self.set_motor_position()
        else:
            self.get_logger().info(f"invalid command type set")
            self.stop_motors()
        self.update_motors()

    def update_motors(self):
        self.front_left_motor.update()
        self.front_right_motor.update()
        self.back_left_motor.update()
        self.back_right_motor.update()

    def mix_inputs(self):
        sig = 0
        front_left_wheel_speed = 0
        front_right_wheel_speed = 0
        back_left_wheel_speed = 0
        back_right_wheel_speed = 0

        if self.command_vel_x > 0 or self.command_vel_x < 0:
            sig += 1
            front_left_wheel_speed += self.command_vel_x
            front_right_wheel_speed += self.command_vel_x
            back_left_wheel_speed += self.command_vel_x
            back_right_wheel_speed += self.command_vel_x

        if self.command_vel_y > 0 or self.command_vel_y < 0:
            sig += 1
            front_left_wheel_speed += self.command_vel_y
            front_right_wheel_speed -= self.command_vel_y
            back_left_wheel_speed -= self.command_vel_y
            back_right_wheel_speed += self.command_vel_y

        if self.command_vel_t > 0 or self.command_vel_t < 0:
            sig += 1
            front_left_wheel_speed += self.command_vel_t
            front_right_wheel_speed -= self.command_vel_t
            back_left_wheel_speed += self.command_vel_t
            back_right_wheel_speed -= self.command_vel_t

        if sig != 0:
            front_left_wheel_speed = front_left_wheel_speed / sig
            front_right_wheel_speed = front_right_wheel_speed / sig
            back_left_wheel_speed = back_left_wheel_speed / sig
            back_right_wheel_speed = back_right_wheel_speed / sig

        self.front_left_wheel_speed = front_left_wheel_speed
        self.front_right_wheel_speed = front_right_wheel_speed
        self.back_left_wheel_speed = back_left_wheel_speed
        self.back_right_wheel_speed = back_right_wheel_speed

    def max_inputs(self):

        if self.command_vel_x > self.command_vel_y and self.command_vel_x > self.command_vel_t:
            self.front_left_wheel_speed += self.command_vel_x
            self.front_right_wheel_speed += self.command_vel_x
            self.back_left_wheel_speed += self.command_vel_x
            self.back_right_wheel_speed += self.command_vel_x

        elif self.command_vel_y > self.command_vel_t:
            self.front_left_wheel_speed += self.command_vel_y
            self.front_right_wheel_speed -= self.command_vel_y
            self.back_left_wheel_speed -= self.command_vel_y
            self.back_right_wheel_speed += self.command_vel_y
        else:
            self.front_left_wheel_speed += self.command_vel_t
            self.front_right_wheel_speed -= self.command_vel_t
            self.back_left_wheel_speed += self.command_vel_t
            self.back_right_wheel_speed -= self.command_vel_t

    def set_motor_speed(self):
        self.set_parameters([Parameter('command_type', Parameter.Type.STRING, "speed")])
        if self.motors_enabled:
            self.get_logger().info(f'updating motor speeds fl:{int(self.front_left_wheel_speed)} fr:'
                                   f'{int(self.front_right_wheel_speed)} bl:{int(self.back_left_wheel_speed)} '
                                   f'br:{int(self.back_right_wheel_speed)}')
            self.front_left_motor.set_speed(self.front_left_wheel_speed)
            self.front_right_motor.set_speed(self.front_right_wheel_speed)
            self.back_left_motor.set_speed(self.back_left_wheel_speed)
            self.back_right_motor.set_speed(self.back_right_wheel_speed)
        else:
            self.get_logger().info('motors disabled')
            self.front_left_motor.set_speed(0)
            self.front_right_motor.set_speed(0)
            self.back_left_motor.set_speed(0)
            self.back_right_motor.set_speed(0)

    def set_motor_position(self):
        self.set_parameters([Parameter('command_type', Parameter.Type.STRING, "pos")])
        self.get_logger().info(f'updating motor positions fl:{self.front_left_wheel_posi} fr:'
                               f'{self.front_right_wheel_posi} bl:{self.back_left_wheel_posi} '
                               f'br:{self.back_right_wheel_posi}')
        self.front_left_motor.set_position(self.front_left_wheel_posi)
        self.front_right_motor.set_position(self.front_right_wheel_posi)
        self.back_left_motor.set_position(self.back_left_wheel_posi)
        self.back_right_motor.set_position(self.back_right_wheel_posi)

    def set_motor_velocity(self):
        self.set_parameters([Parameter('command_type', Parameter.Type.STRING, "vel")])
        if self.motors_enabled:
            self.get_logger().info(f'updating motor velocities fl:{self.front_left_wheel_speed} fr:'
                                   f'{self.front_right_wheel_speed} bl:{self.back_left_wheel_speed} '
                                   f'br:{self.back_right_wheel_speed}')
            self.front_left_motor.set_velocity(self.front_left_wheel_speed)
            self.front_right_motor.set_velocity(self.front_right_wheel_speed)
            self.back_left_motor.set_velocity(self.back_left_wheel_speed)
            self.back_right_motor.set_velocity(self.back_right_wheel_speed)
        else:
            self.get_logger().info('motors disabled')
            self.front_left_motor.set_speed(0)
            self.front_right_motor.set_speed(0)
            self.back_left_motor.set_speed(0)
            self.back_right_motor.set_speed(0)

    def calibrate_motors(self, request, response):
        self.get_logger().info("Motor Calibration started, each motor will move slightly")
        self.front_left_motor.calibrate()
        self.front_right_motor.calibrate()
        self.back_left_motor.calibrate()
        self.back_right_motor.calibrate()
        self.motors_calibrated = True
        self.get_logger().info("motor calibration complete")
        return response

    def disable_motors(self, request, response):
        self.motors_enabled = False
        return response

    def enable_motors(self, request, response):
        self.motors_enabled = True
        return response

    def drive_forward(self, request, response):
        self.get_logger().info(f"drive forward {request.x}")

        # TODO: determine scaling factor from wheel pos to distance
        scale = 10

        self.front_left_wheel_pos = self.front_left_motor.pos
        self.front_right_wheel_pos = self.front_right_motor.pos
        self.back_left_wheel_pos = self.back_left_motor.pos
        self.back_right_wheel_pos = self.back_right_motor.pos

        self.front_left_wheel_posi = self.front_left_wheel_pos + (request.x * scale)
        self.front_right_wheel_posi = self.front_right_wheel_pos + (request.x * scale)
        self.back_left_wheel_posi = self.back_left_wheel_pos + (request.x * scale)
        self.back_right_wheel_posi = self.back_right_wheel_pos + (request.x * scale)

        self.set_motor_position()

        return response

    def drive_backward(self, request, response):
        self.get_logger().info(f"drive backward {request.x}")

        # TODO: determine scaling factor from wheel pos to distance
        scale = 10

        self.front_left_wheel_pos = self.front_left_motor.pos
        self.front_right_wheel_pos = self.front_right_motor.pos
        self.back_left_wheel_pos = self.back_left_motor.pos
        self.back_right_wheel_pos = self.back_right_motor.pos

        self.front_left_wheel_posi = self.front_left_wheel_pos - (request.x * scale)
        self.front_right_wheel_posi = self.front_right_wheel_pos - (request.x * scale)
        self.back_left_wheel_posi = self.back_left_wheel_pos - (request.x * scale)
        self.back_right_wheel_posi = self.back_right_wheel_pos - (request.x * scale)

        self.set_motor_position()

        return response

    def drive_left(self, request, response):
        self.get_logger().info(f"drive left {request.x}")

        # TODO: determine scaling factor from wheel pos to distance
        scale = 10

        self.front_left_wheel_pos = self.front_left_motor.pos
        self.front_right_wheel_pos = self.front_right_motor.pos
        self.back_left_wheel_pos = self.back_left_motor.pos
        self.back_right_wheel_pos = self.back_right_motor.pos

        self.front_left_wheel_posi = self.front_left_wheel_pos - (request.x * scale)
        self.front_right_wheel_posi = self.front_right_wheel_pos + (request.x * scale)
        self.back_left_wheel_posi = self.back_left_wheel_pos + (request.x * scale)
        self.back_right_wheel_posi = self.back_right_wheel_pos - (request.x * scale)

        self.set_motor_position()

        return response

    def drive_right(self, request, response):
        self.get_logger().info(f"drive right {request.x}")

        # TODO: determine scaling factor from wheel pos to distance
        scale = 10

        self.front_left_wheel_pos = self.front_left_motor.pos
        self.front_right_wheel_pos = self.front_right_motor.pos
        self.back_left_wheel_pos = self.back_left_motor.pos
        self.back_right_wheel_pos = self.back_right_motor.pos

        self.front_left_wheel_posi = self.front_left_wheel_pos + (request.x * scale)
        self.front_right_wheel_posi = self.front_right_wheel_pos - (request.x * scale)
        self.back_left_wheel_posi = self.back_left_wheel_pos - (request.x * scale)
        self.back_right_wheel_posi = self.back_right_wheel_pos + (request.x * scale)

        self.set_motor_position()

        return response

    def turn_cw(self, request, response):
        self.get_logger().info(f"drive cw {request.x}")

        # TODO: determine scaling factor from wheel pos to degrees
        scale = 10

        self.front_left_wheel_pos = self.front_left_motor.pos
        self.front_right_wheel_pos = self.front_right_motor.pos
        self.back_left_wheel_pos = self.back_left_motor.pos
        self.back_right_wheel_pos = self.back_right_motor.pos

        self.front_left_wheel_posi = self.front_left_wheel_pos + (request.x * scale)
        self.front_right_wheel_posi = self.front_right_wheel_pos - (request.x * scale)
        self.back_left_wheel_posi = self.back_left_wheel_pos + (request.x * scale)
        self.back_right_wheel_posi = self.back_right_wheel_pos - (request.x * scale)

        self.set_motor_position()

        return response

    def turn_ccw(self, request, response):
        self.get_logger().info(f"drive ccw {request.x}")

        # TODO: determine scaling factor from wheel pos to degrees
        scale = 10

        self.front_left_wheel_pos = self.front_left_motor.pos
        self.front_right_wheel_pos = self.front_right_motor.pos
        self.back_left_wheel_pos = self.back_left_motor.pos
        self.back_right_wheel_pos = self.back_right_motor.pos

        self.front_left_wheel_posi = self.front_left_wheel_pos - (request.x * scale)
        self.front_right_wheel_posi = self.front_right_wheel_pos + (request.x * scale)
        self.back_left_wheel_posi = self.back_left_wheel_pos - (request.x * scale)
        self.back_right_wheel_posi = self.back_right_wheel_pos + (request.x * scale)

        self.set_motor_position()

        return response

    def stop_motors(self):
        self.front_left_motor.set_speed(0)
        self.front_right_motor.set_speed(0)
        self.back_left_motor.set_speed(0)
        self.back_right_motor.set_speed(0)
        self.update_motors()

    def set_pwm_freq(self, freq):
        self.front_left_motor.set_pwm_freq(freq)
        self.front_right_motor.set_pwm_freq(freq)
        self.back_left_motor.set_pwm_freq(freq)
        self.back_right_motor.set_pwm_freq(freq)


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
