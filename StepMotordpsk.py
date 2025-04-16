import time
import struct
import serial
import RPi.GPIO as GPIO
from sklearn.preprocessing import PolynomialFeatures, StandardScaler
import pandas as pd
import joblib
import numpy as np

# GPIO Setup
driverPUL = 12
driverDIR = 18
Steps = 800

GPIO.setmode(GPIO.BCM)
GPIO.setup(driverPUL, GPIO.OUT)
GPIO.setup(driverDIR, GPIO.OUT)

# Serial Setup
ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=5)
ser.reset_input_buffer()

class StepperMotor:
    def __init__(self, step_pin, dir_pin):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.velocity = 100
        self.accel = 10

    def move(self, steps):
        direction = GPIO.HIGH if steps > 0 else GPIO.LOW
        GPIO.output(self.dir_pin, direction)
        
        delay = 1.0 / self.velocity
        for _ in range(abs(steps)):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)

# ------ 参数接收改进部分 ------
def read_exact(ser, num_bytes):
    data = bytearray()
    while len(data) < num_bytes:
        chunk = ser.read(num_bytes - len(data))
        if not chunk:
            raise TimeoutError("Serial read timeout")
        data.extend(chunk)
    return bytes(data)

# 接收标定指令
calib_cmd = ser.readline().decode().strip()
if calib_cmd == 'Yes':
    while True:
        cmd = struct.unpack('<H', read_exact(ser, 2))[0]
        if cmd == 3:
            break
        elif cmd in [1, 2]:
            motor.move(1 if cmd ==1 else -1)

# 接收实验参数
N = struct.unpack('<H', read_exact(ser, 2))[0]
iterations = struct.unpack('<H', read_exact(ser, 2))[0]
error_rate = struct.unpack('<f', read_exact(ser, 4))[0]
tip_type = ser.readline().decode().strip()

# 加载对应模型
if tip_type == 'black':
    model = joblib.load('blackTip_model.pkl')
else:
    model = joblib.load('plasticTip_model.pkl')

motor = StepperMotor(driverPUL, driverDIR)

for _ in range(N):
    desired_force = struct.unpack('<H', read_exact(ser, 2))[0]
    
    # 预测速度并运动
    predicted_speed = model.predict(...)  # 根据实际模型调整
    motor.velocity = predicted_speed
    motor.move(Steps)
    motor.move(-Steps)
    
    # 等待力数据
    while True:
        if ser.in_waiting >=4:
            actual_force = struct.unpack('<f', read_exact(ser,4))[0]
            error = (actual_force - desired_force)/desired_force
            if abs(error) <= error_rate/100:
                ser.write(b'\x01')  # 二进制确认
                break
            else:
                ser.write(b'\x02') 

GPIO.cleanup()
