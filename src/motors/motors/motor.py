"""
Motor
Ian Sodersjerna
"""
import lgpio
import time
from numpy import interp
from simple_pid import PID


class Motor:
    """
    Represents a motor w/encoder connected via a H-bridge motor driver.
    """
    FREQ = 500

    def __init__(self, gpio, motor1_pin, motor2_pin, speed_pin, encoder1_pin, encoder2_pin,
                 vel_pid_p=0.04, vel_pid_i=0.5, vel_pid_d=0,
                 pos_pid_p=1, pos_pid_i=0.5, pos_pid_d=0.0):
        """
        Initialize the pid controllers, motor pins and variables for motor

        :param gpio: lgpio handle
        :param motor1_pin: Direction pin 1
        :param motor2_pin: Direction pin 2
        :param speed_pin: Speed control pin
        :param encoder1_pin: Encoder pin A
        :param encoder2_pin: Encoder pin B
        """

        # save gpio handle
        self.gpio = gpio

        # velocity based PID controller
        self.vel_controlled = False
        self.vel_p_value = vel_pid_p
        self.vel_i_value = vel_pid_i
        self.vel_d_value = vel_pid_d
        self.velocity_controller = PID(self.vel_p_value, self.vel_i_value, self.vel_d_value, setpoint=0)
        self.velocity_controller.output_limits = (-100, 100)
        self.min_velocity = 0  # the minimum constant velocity the motor can achieve
        self.max_velocity = 0  # the maximum constant velocity the motor can achieve
        # TODO: find min and max velocities of each motor, then find scaling factor to make each motor behave similarly.
        #   then find scaling factor between expected distance traveled and actual distance traveled (this
        #   must be an average because of uncertainty) multiply output by ratio (expected/actual)

        # Positional Based PID controller
        self.pos_controlled = False
        self.pos_p_value = pos_pid_p
        self.pos_i_value = pos_pid_i
        self.pos_d_value = pos_pid_d
        self.pos_setpoint = 0
        self.positional_controller = PID(self.pos_p_value, self.pos_i_value, self.pos_d_value, setpoint=0)
        self.positional_controller.output_limits = (-100, 100)

        self.speed_controlled = False
        # motor characteristics
        self.min_pwm = 0  # represents the minimum pwm value required to move a motor (not used)

        # motor variables
        self.pos = 0
        self.speed = 0
        self.last_pos = 0
        self.last_vel = 0
        self.last_acc = 0

        # pin assignments
        self.motorA_pin = motor1_pin
        self.motorB_pin = motor2_pin
        self.speed_pin = speed_pin
        self.encoderA_pin = encoder1_pin
        self.encoderB_pin = encoder2_pin

        # encoder variables
        self.lastGpio = -1
        self.levA = 0
        self.levB = 0

        # Claim motor outputs.
        lgpio.gpio_claim_output(self.gpio, self.motorA_pin)
        lgpio.gpio_claim_output(self.gpio, self.motorB_pin)
        lgpio.gpio_claim_output(self.gpio, self.speed_pin)
        lgpio.tx_pwm(self.gpio, self.speed_pin, self.FREQ, 0)

        # Claim PID inputs and setup callbacks
        lgpio.gpio_claim_alert(self.gpio, self.encoderA_pin, lgpio.BOTH_EDGES)
        lgpio.gpio_claim_alert(self.gpio, self.encoderB_pin, lgpio.BOTH_EDGES)
        self.gpio_callbackA = lgpio.callback(self.gpio, self.encoderA_pin, lgpio.BOTH_EDGES, self._pulse)
        self.gpio_callbackB = lgpio.callback(self.gpio, self.encoderB_pin, lgpio.BOTH_EDGES, self._pulse)

        # initialization time
        self.last_time = time.perf_counter()

    def __del__(self):
        """
        Ensure the motor is stopped

        :return:
        """
        self._set_speed(0)
        self.cancel()

    def _pulse(self, chip, gpio, level, tick):
        """
        Decode the rotary encoder pulse.

        :param chip:
        :param gpio:
        :param level:
        :param tick:
        :return: None
        =================================================
                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
        """
        if gpio == self.encoderA_pin:
            self.levA = level
        else:
            self.levB = level
        if gpio != self.lastGpio:  # debounce
            self.lastGpio = gpio
            if gpio == self.encoderA_pin and level == 1:
                if self.levB == 1:
                    self.pos += 1
            elif gpio == self.encoderB_pin and level == 1:
                if self.levA == 1:
                    self.pos -= 1

    def cancel(self):
        """
        Cancel the rotary encoder decoder.

        :return: None
        """
        self.gpio_callbackA.cancel()
        self.gpio_callbackB.cancel()

    def calibrate(self):
        """
        This function calibrates a single motor by itself.

        :return: None
        """
        # stop the motor
        self._set_speed(0)
        time.sleep(1)

        current_pos = self.pos

        time.sleep(1)
        if current_pos != self.pos:
            raise RuntimeError("Motor is running cannot begin calibration")

        min_pwm = 0
        while current_pos == self.pos:
            min_pwm += 1
            self._set_speed(min_pwm)
            time.sleep(0.1)
        self._set_speed(0)
        self.min_pwm = min_pwm
        print("min_pwm:", self.min_pwm)

    def update(self):
        """
        Calculate the velocity and acceleration of the motor then set the speed.

        :return: None
        """
        # Calculate change in time
        t1 = time.perf_counter()  # time in seconds
        dt = t1 - self.last_time

        if dt > 10:
            print("time differential between update calls greater than 10 seconds")
            # raise Warning("time differential between update calls greater than 10 seconds")

        # Calculate Velocity and acceleration
        vel = (self.pos - self.last_pos) / dt
        acc = (vel - self.last_vel) / dt

        # update variables
        self.last_pos = self.pos
        self.last_vel = vel
        self.last_acc = acc
        self.last_time = t1

        if self.pos_controlled:
            if abs(self.positional_controller.setpoint - self.pos) > 10:
                self._set_speed(self.positional_controller(self.pos))
            else:
                self._set_speed(0)
        elif self.vel_controlled:
            self._set_speed(self.velocity_controller(vel))
        elif self.speed_controlled:
            self._set_speed(self.speed)
        else:
            print("update called but motor has not received a position or velocity command")

    def set_speed(self, speed):
        """
        Set the desired motor speed and adjust flags accordingly.
        :param speed:
        :return:
        """
        self.speed_controlled = True
        self.pos_controlled = False
        self.vel_controlled = False
        self.speed = speed

    def _set_speed(self, speed):
        """
        Private set speed, sets the raw speed of the motor.

        :param speed: speed the motor should be set to (-100 - 100)
        """
        if speed == 0:
            lgpio.gpio_write(self.gpio, self.motorA_pin, 0)
            lgpio.gpio_write(self.gpio, self.motorB_pin, 0)
            lgpio.tx_pwm(self.gpio, self.speed_pin, self.FREQ, 0)
        elif speed > 0:
            scaled_speed = interp(min(speed, 100), [0, 100], [self.min_pwm, 100])
            lgpio.gpio_write(self.gpio, self.motorA_pin, 1)
            lgpio.gpio_write(self.gpio, self.motorB_pin, 0)
            lgpio.tx_pwm(self.gpio, self.speed_pin, self.FREQ, scaled_speed)
        else:
            scaled_speed = interp(min(abs(speed), 100), [0, 100], [self.min_pwm, 100])
            lgpio.gpio_write(self.gpio, self.motorA_pin, 0)
            lgpio.gpio_write(self.gpio, self.motorB_pin, 1)
            lgpio.tx_pwm(self.gpio, self.speed_pin, self.FREQ, scaled_speed)

    def set_position(self, pos):
        """
        Set the setpoint for the positional PID controller

        :param pos: Position the PID controller should try to achieve.
        :return: None
        """
        self.speed_controlled = False
        self.pos_controlled = True
        self.vel_controlled = False
        self.last_time = time.perf_counter()
        self.positional_controller.setpoint = pos

    def set_velocity(self, vel):
        """
        Set the setpoint for the velocity PID controller.

        :param vel: Velocity the PID controller should try to achieve.
        :return: None
        """
        self.speed_controlled = False
        self.pos_controlled = False
        self.vel_controlled = True
        self.last_time = time.perf_counter()
        self.velocity_controller.setpoint = vel

    def set_vel_pid_values(self, p=None, i=None, d=None):
        """
        Set PID variables for the velocity PID controller.

        :param p: Proportional variable for the velocity PID controller
        :param i: Integral variable for the velocity PID controller
        :param d: Derivative variable for the velocity PID controller
        :return: None
        """
        if p:
            self.vel_p_value = p
            self.velocity_controller.Kp = self.vel_p_value
        if i:
            self.vel_i_value = i
            self.velocity_controller.Ki = self.vel_i_value
        if d:
            self.vel_d_value = d
            self.velocity_controller.Kd = self.vel_d_value

    def set_pos_pid_values(self, p=None, i=None, d=None):
        """
        Set PID variables for the positional PID controller.

        :param p: Proportional variable for the positional PID controller
        :param i: Integral variable for the positional PID controller
        :param d: Derivative variable for the positional PID controller
        :return: None
        """
        if p:
            self.pos_p_value = p
            self.positional_controller.Kp = self.pos_p_value
        if i:
            self.pos_i_value = i
            self.positional_controller.Ki = self.pos_i_value
        if d:
            self.pos_d_value = d
            self.positional_controller.Kd = self.pos_d_value

    def set_pwm_freq(self, freq):
        """
        Set the pwm frequency for the motors.

        :param freq: frequency to set the motors to
        :return: None
        """
        self.FREQ = freq


if __name__ == "__main__":
    """
    Test Program
    """
    import argparse

    parser = argparse.ArgumentParser(
        prog='Motor Test Program',
        description='A test program for the motor driver',
        epilog='This program is useful for setting PID variables')

    parser.add_argument('-p', '--pid_p', default=0.04, type=float)
    parser.add_argument('-i', '--pid_i', default=0.5, type=float)
    parser.add_argument('-d', '--pid_d', default=0.0, type=float)

    parser.add_argument('-f', '--pwm_freq', default=25, type=float)

    args = parser.parse_args()

    test_gpio = lgpio.gpiochip_open(0)
    try:
        test_motor = Motor(test_gpio, 18, 24, 17, 25, 5, vel_pid_p=args.pid_p, vel_pid_i=args.pid_i,
                           vel_pid_d=args.pid_d)  # front left
        test_motor2 = Motor(test_gpio, 6, 16, 19, 20, 12, vel_pid_p=args.pid_p, vel_pid_i=args.pid_i,
                            vel_pid_d=args.pid_d)  # front right
        test_motor3 = Motor(test_gpio, 23, 22, 4, 21, 27, vel_pid_p=args.pid_p, vel_pid_i=args.pid_i,
                            vel_pid_d=args.pid_d)  # back left
        test_motor4 = Motor(test_gpio, 13, 26, 11, 9, 10, vel_pid_p=args.pid_p, vel_pid_i=args.pid_i,
                            vel_pid_d=args.pid_d)  # back right

        # test_motor.calibrate()
        # test_motor2.calibrate()
        # test_motor3.calibrate()
        # test_motor4.calibrate()

        sv = 200
        test_motor.set_velocity(sv)
        test_motor2.set_velocity(sv)
        test_motor3.set_velocity(sv)
        test_motor4.set_velocity(sv)

        ss = 100
        # test_motor.set_speed(ss)
        # test_motor2.set_speed(ss)
        # test_motor3.set_speed(ss)
        # test_motor4.set_speed(ss)

        t0 = time.perf_counter()
        # i = 0
        while True:
            test_motor.update()
            test_motor2.update()
            test_motor3.update()
            test_motor4.update()

            print(f"m1_vel: {test_motor.last_vel:3.2f} m2_vel: {test_motor2.last_vel:3.2f}  m3_vel: "
                  f"{test_motor2.last_vel:3.2f} m4_vel: {test_motor3.last_vel:3.2f}")
            time.sleep(0.01)
            break
            # i += 1
            # if i >= 100:
            #     print(f"m1_vel: {test_motor.last_vel:3.2f} m2_vel: {test_motor2.last_vel:3.2f}  m3_vel: "
            #           f"{test_motor2.last_vel:3.2f} m4_vel: {test_motor3.last_vel:3.2f}")
            #     i = 0

            # if time.perf_counter() - t0 > 60:
            #     print("goodbye")
            #     break

        test_motor.set_speed(0)
        test_motor2.set_speed(0)
        test_motor3.set_speed(0)
        test_motor4.set_speed(0)

    except KeyboardInterrupt as e:
        print(e)
        test_motor.set_speed(0)
        test_motor2.set_speed(0)
        test_motor3.set_speed(0)
        test_motor4.set_speed(0)
    finally:
        lgpio.gpiochip_close(test_gpio)
