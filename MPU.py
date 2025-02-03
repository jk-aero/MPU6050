from machine import I2C, Pin
import time
from math import atan, degrees

class KalmanFilter:
    def __init__(self, KalmanState, KalmanUncertainty):
        self.KalmanState = KalmanState
        self.KalmanUncertainty = KalmanUncertainty
        print('inside  kalman')

    def update(self, Gyro, Acc, dt):
        self.KalmanState = self.KalmanState + dt*Gyro
        self.KalmanUncertainty = self.KalmanUncertainty + dt*dt*4*4
        KalmanGain = self.KalmanUncertainty * (1/1*self.KalmanUncertainty + 3*3)
        self.KalmanState = self.KalmanState + KalmanGain*(Acc-self.KalmanState)
        self.KalmanUncertainty = (1-KalmanGain)*self.KalmanUncertainty
        return self.KalmanState

'''================================================================================================================================================='''

class MPU6050:
    def __init__(self, busid, SDA, SCL, led=25):
        self.PWR_MGMT_1 = 0x6B
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H = 0x43
        self.GYRO_YOUT_H = 0x45
        self.GYRO_ZOUT_H = 0x47
        self.mpu6050_addr = 0x68

        self.x_gyro_bias = 0
        self.y_gyro_bias = 0
        self.z_gyro_bias = 0

        self.x_acc_bias = 0
        self.y_acc_bias = 0

        self.KalmanStateX = 0
        self.KalmanUncertaintyX = 0
        self.pitchAngle = KalmanFilter(self.KalmanStateX, self.KalmanUncertaintyX)

        self.KalmanStateY = 0
        self.KalmanUncertaintyY = 0
        self.rollAngle = KalmanFilter(self.KalmanStateY, self.KalmanUncertaintyY)

        self.LED = Pin(led, Pin.OUT)

        self.i2c = I2C(busid, sda=Pin(SDA), scl=Pin(SCL))
        self.i2c.writeto_mem(self.mpu6050_addr, self.PWR_MGMT_1, b'\x01')

    def _combine_register_values_(self, h, l):
        if not h[0] & 0x80:
            return h[0] << 8 | l[0]
        return -((h[0] ^ 255) << 8) | (l[0] ^ 255) + 1

    def _read_raw_data_(self, addr):
        high = self.i2c.readfrom_mem(self.mpu6050_addr, addr, 1)
        low = self.i2c.readfrom_mem(self.mpu6050_addr, addr+1, 1)
        val = self._combine_register_values_(high, low)
        return (val)

    def read_acc(self):
        return (self._read_raw_data_(0x3B)/16384, self._read_raw_data_(0x3D)/16384, self._read_raw_data_(0x3F)/16384)

    def read_gyro(self):
        return (self._read_raw_data_(0x43)/131-1.01747575, self._read_raw_data_(0x45)/131+1.31162225, self._read_raw_data_(0x47)/131-1.91204825)

    def blink(self, t):
        for i in range(6):
            self.LED.toggle()
            time.sleep(t)

    def calculate_acc_angles(self):
        x, y, z = [], [], []
        for i in range(3):
            acc_data = self.read_acc()
            x.append(acc_data[0])
            y.append(acc_data[1])
            z.append(acc_data[2])
        ax = sum(x)/3
        ay = sum(y)/3
        az = sum(z)/3
        x_angles = degrees(atan(ay/((ax**2 + az**2)**0.5)))
        y_angles = degrees(atan(ax/((ay**2 + az**2)**0.5)))-5.18
        return (x_angles-self.x_acc_bias, y_angles-self.y_acc_bias)

    def callibrate_gyro(self):
        GX_bias, GY_bias, GZ_bias = [], [], []
        for i in range(100):
            g_x, g_y, g_z = self.read_gyro()
            GX_bias.append(g_x)
            GY_bias.append(g_y)
            GZ_bias.append(g_z)
        self.z_gyro_bias = sum(GX_bias)/100
        self.y_gyro_bias = sum(GY_bias)/100
        self.z_gyro_bias = sum(GZ_bias)/100
        del GX_bias, GY_bias, GZ_bias
        self.blink(0.1)
        time.sleep(2)

    def callibrate_acc(self):
        lstX, lstY = [], []
        for i in range(100):
            a_x, a_y, a_z = self.read_acc()
            x_angle, y_angle = self.calculate_acc_angles()
            lstX.append(x_angle)
            lstY.append(y_angle)

        self.x_acc_bias = (sum(lstX)/100)
        self.y_acc_bias = (sum(lstY)/100)
        del lstX, lstY
        self.blink(0.1)
        time.sleep(2)

    def return_angles(self):
        start = time.ticks_us()
        gX, gY = self.read_gyro()[:-1]
        Acc_X, Acc_Y = self.calculate_acc_angles()
        dt = time.ticks_diff(time.ticks_us(), start)/10**6
        return self.pitchAngle.update(gY, Acc_Y, dt), self.rollAngle.update(gX, Acc_X, dt), dt
