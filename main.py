from mpu import MPU6050
import time

obj = MPU6050(0,12,13)  # first parameter is busid ,2nd parameter SDA pin,3rd parameter SCL pin,4th parameter LED pin for indication
obj.callibrate_gyro() #calibrate gyro 
obj.callibrate_acc() #calibrating accelerometer

"""put below in loop so that it will give output continuously"""
#obj.read_acc() # will fetch raw values from accelerometer

while True:
  obj.return_angles() #will give alist containing pitch and roll angles (filtered with kalmanfilter)
