import lgpio


class Motor:

    FREQ = 10000

    def __init__(self, gpio, motor1_pin, motor2_pin, speed_pin, encoder1_pin, encoder2_pin):

        self.gpio = gpio

        self.pos = 0

        self.motorA_pin = motor1_pin
        self.motorB_pin = motor2_pin
        self.speed_pin = speed_pin
        self.encoderA_pin = encoder1_pin
        self.encoderB_pin = encoder2_pin

        self.lastGpio = -1
        self.levA = 0
        self.levB = 0

        lgpio.gpio_claim_output(self.gpio, self.motorA_pin)
        lgpio.gpio_claim_output(self.gpio, self.motorB_pin)
        lgpio.gpio_claim_output(self.gpio, self.speed_pin)
        lgpio.tx_pwm(self.gpio, self.speed_pin, self.FREQ, 0)

        lgpio.gpio_claim_alert(self.gpio, self.encoderA_pin, lgpio.BOTH_EDGES)
        lgpio.gpio_claim_alert(self.gpio, self.encoderB_pin, lgpio.BOTH_EDGES)
        self.gpio_callbackA = lgpio.callback(self.gpio, self.encoderA_pin, lgpio.BOTH_EDGES, self._pulse)
        self.gpio_callbackB = lgpio.callback(self.gpio, self.encoderB_pin, lgpio.BOTH_EDGES, self._pulse)

    def set_speed(self, speed):
        if speed == 0:
            lgpio.gpio_write(self.gpio, self.motorA_pin, 0)
            lgpio.gpio_write(self.gpio, self.motorB_pin, 0)
            lgpio.tx_pwm(self.gpio, self.speed_pin, self.FREQ, 0)
        elif speed > 0:
            lgpio.gpio_write(self.gpio, self.motorA_pin, 1)
            lgpio.gpio_write(self.gpio, self.motorB_pin, 0)
            lgpio.tx_pwm(self.gpio, self.speed_pin, self.FREQ, min(speed, 100))
        else:
            lgpio.gpio_write(self.gpio, self.motorA_pin, 0)
            lgpio.gpio_write(self.gpio, self.motorB_pin, 1)
            lgpio.tx_pwm(self.gpio, self.speed_pin, self.FREQ, min(abs(speed), 100))

    def _pulse(self, chip, gpio, level, tick):
        """
        Decode the rotary encoder pulse.

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
        """
        self.gpio_callbackA.cancel()
        self.gpio_callbackB.cancel()


if __name__ == "__main__":
    import time

    test_gpio = lgpio.gpiochip_open(0)
    try:
        print("Start")
        test_motor = Motor(test_gpio, 18, 24, 17, 25, 5)  # front left
        test_motor2 = Motor(test_gpio, 6, 16, 19, 20, 12)  # front right
        test_motor3 = Motor(test_gpio, 23, 22, 4, 21, 27)  # back left
        test_motor4 = Motor(test_gpio, 13, 26, 11, 9, 10)  # back right

        # forward
        test_motor.set_speed(25)
        test_motor2.set_speed(25)
        test_motor3.set_speed(25)
        test_motor4.set_speed(25)
        time.sleep(2)
        test_motor.set_speed(50)
        test_motor2.set_speed(50)
        test_motor3.set_speed(50)
        test_motor4.set_speed(50)
        time.sleep(2)
        test_motor.set_speed(75)
        test_motor2.set_speed(75)
        test_motor3.set_speed(75)
        test_motor4.set_speed(75)
        time.sleep(2)
        test_motor.set_speed(100)
        test_motor2.set_speed(100)
        test_motor3.set_speed(100)
        test_motor4.set_speed(100)
        time.sleep(2)

        # stop
        test_motor.set_speed(0)
        test_motor2.set_speed(0)
        test_motor3.set_speed(0)
        test_motor4.set_speed(0)
        time.sleep(2)

        # backwards
        test_motor.set_speed(-25)
        test_motor2.set_speed(-25)
        test_motor3.set_speed(-25)
        test_motor4.set_speed(-25)
        time.sleep(2)
        test_motor.set_speed(-50)
        test_motor2.set_speed(-50)
        test_motor3.set_speed(-50)
        test_motor4.set_speed(-50)
        time.sleep(2)
        test_motor.set_speed(-75)
        test_motor2.set_speed(-75)
        test_motor3.set_speed(-75)
        test_motor4.set_speed(-75)
        time.sleep(2)
        test_motor.set_speed(-100)
        test_motor2.set_speed(-100)
        test_motor3.set_speed(-100)
        test_motor4.set_speed(-100)
        time.sleep(2)

        # stop
        test_motor.set_speed(0)
        test_motor2.set_speed(0)
        test_motor3.set_speed(0)
        test_motor4.set_speed(0)
        time.sleep(2)

    except KeyboardInterrupt as e:
        print(e)
    finally:
        lgpio.gpiochip_close(test_gpio)
