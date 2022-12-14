import time
import lgpio
from motor import Motor

test_gpio = lgpio.gpiochip_open(0)

test_motor = Motor(test_gpio, 18, 24, 17, 25, 5)  # front left
test_motor2 = Motor(test_gpio, 6, 16, 19, 20, 12)  # front right
test_motor3 = Motor(test_gpio, 23, 22, 4, 21, 27)  # back left
test_motor4 = Motor(test_gpio, 13, 26, 11, 9, 10)  # back right

test_motor.set_speed(0)
test_motor2.set_speed(0)
test_motor3.set_speed(0)
test_motor4.set_speed(0)
time.sleep(0.2)

lgpio.gpiochip_close(test_gpio)

