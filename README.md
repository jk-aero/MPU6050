# MPU6050 Sensor Interface with Raspberry Pi Pico

This project demonstrates how to interface an MPU6050 (accelerometer and gyroscope sensor) with a Raspberry Pi Pico using MicroPython. It provides real-time pitch and roll angles using a Kalman filter for improved accuracy.

## Features

- **Raw Accelerometer Data**: Reads raw X, Y, Z values from the accelerometer
- **Raw Gyroscope Data**: Reads raw X, Y, Z values from the gyroscope
- **Accelerometer-Based Angles**: Calculates angles using accelerometer readings
- **Kalman Filtered Angles**: Provides stabilized pitch and roll angles using a Kalman filter
- **Calibration Functions**: Allows calibration of both the gyroscope and accelerometer for better accuracy

## Requirements

- Raspberry Pi Pico (or any compatible microcontroller with I2C support)
- MPU6050 sensor module
- MicroPython firmware installed on the Raspberry Pi Pico

## Wiring

Ensure you connect the MPU6050 module correctly to the Raspberry Pi Pico:


| MPU6050 Pin | Raspberry Pi Pico Pin |
|-------------|-----------------------|
| VCC         | 3.3V                  |
| GND         | GND                   |
| SDA         | GPIO12                |
| SCL         | GPIO13                |


## Installation

1. Install MicroPython on the Raspberry Pi Pico if not already installed
2. Copy the `MPU.py` library to the Pico filesystem
3. Upload the provided script to the Pico
4. Run the script using a MicroPython environment like Thonny

## Usage

### Initialize the Sensor
```python
from MPU import MPU6050

# Create an MPU6050 object (bus ID 0, SDA=GPIO12, SCL=GPIO13)
obj = MPU6050(0, 12, 13)
```


### Calibrate the Sensor
```python
obj.callibrate_gyro()  # Calibrate gyroscope
obj.callibrate_acc()   # Calibrate accelerometer
```
### Read Sensor Data
```python
accel_data = obj.read_acc()  # Get raw accelerometer data [x, y, z]
gyro_data = obj.read_gyro()  # Get raw gyroscope data [x, y, z]
angles = obj.calculate_acc_angles()  # Get angles from accelerometer [x, y]
kf_angles = obj.return_angles()  # Get Kalman filtered angles [pitch, roll]
```

### Continuous Angle Output
```python
while True:
    print(obj.return_angles())  # Print filtered pitch and roll angles
```

### Example Output
Pitch: 12.34°, Roll: -5.67°
Pitch: 12.31°, Roll: -5.65°
Pitch: 12.29°, Roll: -5.62°

### Notes
Calibration is essential for accurate readings
Ensure correct I2C bus ID based on your microcontroller
Use proper power supply (3.3V for Raspberry Pi Pico, 5V for other microcontrollers if applicable)
Keep the sensor stationary during calibration for best results
The Kalman filter parameters may need tuning for your specific application

### Troubleshooting
If getting I2C errors, verify wiring and try different GPIO pins
Ensure the MPU6050 is properly powered (3.3V for Pico)
Check for proper MicroPython I2C driver installation
If readings are unstable, try recalibrating the sensor
