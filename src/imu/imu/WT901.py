"""
WT901
Ian Sodersjerna
"""
import time
import lgpio
from struct import unpack


class WT901:
    """
    JY901 represents a driver for the WT901 High-Accuracy Accelerometer, Gyroscope, Magnetometer Module.
    """

    class STime:
        """
        Class that represents time from the WT901
        """
        ucYear = ''
        ucMonth = ''
        ucDay = ''
        ucHour = ''
        ucMinute = ''
        ucSecond = ''
        usMiliSecond = 0

    class SAcc:
        """
        Class that represents linear acceleration from the WT901
        """
        ax = 0
        ay = 0
        az = 0
        t = 0

    class SGyro:
        """
        Class that represents gyroscopic velocity from the WT901
        """
        gx = 0
        gy = 0
        gz = 0
        t = 0

    class SAngle:
        """
        Class that represents kalman filtered angle from the WT901
        """
        ax = 0
        ay = 0
        az = 0
        t = 0

    class SMag:
        """
        Class that represents magnetometer readings from the WT901
        """
        mx = 0
        my = 0
        mz = 0
        t = 0

    def __init__(self, addr=0x50):
        """
        Initialize a IMU object
        :param addr: address of the IMU on the i2s buss
        """
        # save the address then open communications with the device
        self.uc_dev_addr = addr
        self.iic_handle = lgpio.i2c_open(1, self.uc_dev_addr)

        # IMU raw variables
        self.stcTime = self.STime
        self.stcAcc = self.SAcc
        self.stcGyro = self.SGyro
        self.stcAngle = self.SAngle
        self.stcMag = self.SMag

        # IMU estimated positions
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0

    def update(self):
        """
        Update the imu
        :return: None
        """
        # update the acceleration
        self.get_acc()

        # update the gyroscope
        self.get_gyro()

        # accumulate
        # TODO: determine a better method than just accumulating
        self.accumulate()

    def accumulate(self):
        """
        accumulate the acceleration
        :return:
        """
        self.x_pos += self.stcAcc.ax
        self.y_pos += self.stcAcc.ay
        self.z_pos += self.stcGyro.gz
        print(f"Estimations x:{self.x_pos} y:{self.y_pos} z:{self.z_pos}")

    def get_time(self):
        """
        Get the time from the IMU
        :return: None
        """
        lgpio.i2c_write_device(self.iic_handle, [0x30])
        (count, data) = lgpio.i2c_read_device(self.iic_handle, 8)
        print(unpack('<BBBBBBH', data))
        self.stcTime.ucYear = data[0]
        self.stcTime.ucMonth = data[1]
        self.stcTime.ucDay = data[2]
        self.stcTime.ucHour = data[3]
        self.stcTime.ucMinute = data[4]
        self.stcTime.ucSecond = data[5]
        self.stcTime.usMiliSecond = data[6]
        # print(f"hour: {self.stcTime.ucHour} minute: {self.stcTime.ucMinute} second: {self.stcTime.ucSecond}")

    def get_acc(self):
        """
        Get the acceleration values from the IMU
        :return: None
        """
        lgpio.i2c_write_device(self.iic_handle, [0x34])
        (count, data) = lgpio.i2c_read_device(self.iic_handle, 6)
        temp = unpack('<hhh', data)
        self.stcAcc.ax = float(temp[0]) / 32768 * 16
        self.stcAcc.ay = float(temp[1]) / 32768 * 16
        self.stcAcc.az = float(temp[2]) / 32768 * 16
        # print(f"acc x: {self.stcAcc.ax} y: {self.stcAcc.ay} z: {self.stcAcc.az}")

    def get_gyro(self):
        """
        Get the gyroscope values from the IMU
        :return: None
        """
        lgpio.i2c_write_device(self.iic_handle, [0x37])
        (count, data) = lgpio.i2c_read_device(self.iic_handle, 6)
        temp = unpack('<hhh', data)
        self.stcGyro.gx = float(temp[0]) / 32768 * 2000
        self.stcGyro.gy = float(temp[1]) / 32768 * 2000
        self.stcGyro.gz = float(temp[2]) / 32768 * 2000
        # print(f"gyro x: {self.stcGyro.gx} y: {self.stcGyro.gy} z: {self.stcGyro.gz}")

    def get_angle(self):
        """
        Get the angular values from the IMU
        :return: None
        """
        lgpio.i2c_write_device(self.iic_handle, [0x3d])
        (count, data) = lgpio.i2c_read_device(self.iic_handle, 6)
        temp = unpack('<hhh', data)
        self.stcAngle.ax = float(temp[0]) / 32768 * 180
        self.stcAngle.ay = float(temp[1]) / 32768 * 180
        self.stcAngle.az = float(temp[2]) / 32768 * 180
        # print(f"angle x: {self.stcAngle.ax} y: {self.stcAngle.ay} z: {self.stcAngle.az}")

    def get_mag(self):
        """
        Get the magnetometer values from the IMU
        :return: None
        """
        lgpio.i2c_write_device(self.iic_handle, [0x3a])
        (count, data) = lgpio.i2c_read_device(self.iic_handle, 6)
        temp = unpack('<hhh', data)
        self.stcMag.mx = float(temp[0]) / 32768 * 180
        self.stcMag.my = float(temp[1]) / 32768 * 180
        self.stcMag.mz = float(temp[2]) / 32768 * 180
        # print(f"angle x: {self.stcMag.mx} y: {self.stcMag.my} z: {self.stcMag.mz}")


if __name__ == "__main__":
    imu = WT901()
    while True:
        # imu.get_time()
        imu.get_acc()
        imu.get_gyro()
        imu.get_angle()
        print()
        time.sleep(1)
