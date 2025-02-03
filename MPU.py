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

    def _bytes_to_signed_16bit_(self, hi, lo):
        return (((hi << 8) | lo) ^ 0x8000) - 0x8000

    def _read_raw_data_(self, addr):
        bytes = self.i2c.readfrom_mem(self.mpu6050_addr, addr, 2)
        return self._bytes_to_signed_16bit_(bytes[0], bytes[1])

    def read_acc(self):
        return (self._read_raw_data_(0x3B)/16384, self._read_raw_data_(0x3D)/16384, self._read_raw_data_(0x3F)/16384)

    def read_gyro(self):
        return (self._read_raw_data_(0x43)/131-1.01747575, self._read_raw_data_(0x45)/131+1.31162225, self._read_raw_data_(0x47)/131-1.91204825)

    def blink(self, t):
        for i in range(6):
            self.LED.toggle()
            time.sleep(t)

    def calculate_acc_angles(self):
        x_sum, y_sum, z_sum = 0, 0, 0
        num_samples = 3
        for i in range(num_samples):
            acc_data = self.read_acc()
            x_sum += acc_data[0]
            y_sum += acc_data[1]
            z_sum += acc_data[2]
        ax = x_sum / num_samples
        ay = y_sum / num_samples
        az = z_sum / num_samples
        x_angles = degrees(atan(ay/((ax**2 + az**2)**0.5)))
        y_angles = degrees(atan(ax/((ay**2 + az**2)**0.5)))-5.18
        return (x_angles-self.x_acc_bias, y_angles-self.y_acc_bias)

    def callibrate_gyro(self):
        GX_sum, GY_sum, GZ_sum = 0, 0, 0
        num_samples = 100
        for i in range(num_samples):
            g_x, g_y, g_z = self.read_gyro()
            GX_sum += g_x
            GY_sum += g_y
            GZ_sum += g_z
        self.z_gyro_bias = GX_sum / num_samples
        self.y_gyro_bias = GY_sum / num_samples
        self.z_gyro_bias = GZ_sum / num_samples
        self.blink(0.1)
        time.sleep(2)

    def callibrate_acc(self):
        x_sum, y_sum = 0, 0
        num_samples = 100
        for i in range(num_samples):
            a_x, a_y, a_z = self.read_acc()
            x_angle, y_angle = self.calculate_acc_angles()
            x_sum += x_angle
            y_sum += y_angle

        self.x_acc_bias = x_sum / num_samples
        self.y_acc_bias = y_sum / num_samples
        self.blink(0.1)
        time.sleep(2)

    def return_angles(self):
        start = time.ticks_us()
        gX, gY = self.read_gyro()[:-1]
        Acc_X, Acc_Y = self.calculate_acc_angles()
        dt = time.ticks_diff(time.ticks_us(), start)/10**6
        return self.pitchAngle.update(gY, Acc_Y, dt), self.rollAngle.update(gX, Acc_X, dt), dt
