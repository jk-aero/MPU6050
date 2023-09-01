from MPU import MPU6050

# create an mpu object
# first parameter is bus Id ,2nd parameter SDA pin,3rd parameter SCL pin,4th parameter LED pin for indication
#look at the microcontroller pinout to find which I2c bus is being used
#4th parameter is set as pin 25 as default ;because pi pico have onboard led on pin 25


obj = MPU6050(0,12,13)  
obj.callibrate_gyro() #calibrate gyro 
obj.callibrate_acc() #calibrating accelerometer

"""put below in loop so that it will give output continuously"""
#obj.read_acc()                       # will return raw values from accelerometer in [x,y,z] order
#obj.read_gyro()                      # will return raw values from gyroscope in [x,y,z] order
#obj.calculate_acc_angles()           #will return angles calculated from accelerometer values in [x,y] order
#obj.return_angles                    #will return kalmanfiltered [x,y] angles




while True:
  obj.return_angles() #will give alist containing pitch and roll angles (filtered with kalmanfilter)
