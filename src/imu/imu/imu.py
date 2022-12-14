"""
fd = uart_open(fd,"/dev/ttyUSB0");

void ParseData(char chr)
{
    static char chrBuf[100];
    static unsigned char chrCnt=0;
    signed short sData[4];
    unsigned char i;
    char cTemp=0;
    time_t now;
    chrBuf[chrCnt++]=chr;
    if (chrCnt<11) return;
    for (i=0;i<10;i++) cTemp+=chrBuf[i];
    if ((chrBuf[0]!=0x55)||((chrBuf[1]&0x50)!=0x50)||(cTemp!=chrBuf[10])) {
        printf("Error:%x %x\r\n",chrBuf[0],chrBuf[1]);
        memcpy(&chrBuf[0],&chrBuf[1],10);
        chrCnt--;
        return;
    }

    memcpy(&sData[0],&chrBuf[2],8);
    switch(chrBuf[1])
        {
        case 0x51:
            for (i=0;i<3;i++) a[i] = (float)sData[i]/32768.0*16.0;
            time(&now);
            printf("\r\nT:%s a:%6.3f %6.3f %6.3f ",asctime(localtime(&now)),a[0],a[1],a[2]);

            break;
        case 0x52:
            for (i=0;i<3;i++) w[i] = (float)sData[i]/32768.0*2000.0;
            printf("w:%7.3f %7.3f %7.3f ",w[0],w[1],w[2]);
            break;
        case 0x53:
            for (i=0;i<3;i++) Angle[i] = (float)sData[i]/32768.0*180.0;
            printf("A:%7.3f %7.3f %7.3f ",Angle[0],Angle[1],Angle[2]);
            break;
        case 0x54:
            for (i=0;i<3;i++) h[i] = (float)sData[i];
            printf("h:%4.0f %4.0f %4.0f ",h[0],h[1],h[2]);
            break;
        }
    chrCnt=0;
}

"""
import lgpio


class IMU:
    count = 1000  # 1000 is default

    def __init__(self):
        self.imu_serial = lgpio.serial_open("/dev/ttyUSB0", 9600)

        self.imu_x = 0
        self.delta_time = 0
        self.x_pos = 0
        self.y_pos = 0
        self.heading = 0

    def __del__(self):
        if self.imu_serial:
            lgpio.serial_close(self.imu_serial)

    def read_serial(self):
        if lgpio.serial_data_available(self.imu_serial):
            (b, d) = lgpio.serial_read(self.imu_serial, self.count)

            if b > 0:
                if d[0] == hex(81):
                    # time
                    print("time: ", d)
                elif d[0] == hex(82):
                    # w
                    print("w: ", d)
                elif d[0] == hex(83):
                    # a
                    print("a: ", d)
                elif d[0] == hex(84):
                    # h
                    print("h: ", d)

    def calculate_odom(self, acc_x, acc_y, acc_t, dt):
        self.x_pos += (acc_x * dt)
        self.y_pos += (acc_y * dt)
        self.heading += (acc_t * dt)


if __name__ == "__main__":
    imu = IMU()
    while True:
        imu.read_serial()
