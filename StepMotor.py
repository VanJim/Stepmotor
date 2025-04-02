import numpy as np
import pandas as pd
import sklearn
import sklearn.preprocessing
import joblib
from time import sleep
import time
import RPi.GPIO as GPIO
import serial
import struct
driverPUL = 12
driverDIR = 18
Steps = 800 
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(driverPUL, GPIO.OUT)
GPIO.setup(driverDIR, GPIO.OUT)
class StepperMotor:
    def __init__(self, step_pin, direction_pin):
        self.step_pin = step_pin
        self.direction_pin = direction_pin
        self.acceleration = 20
        self.max_velocity = 100000
        self.deceleration = 20
        self.current_velocity = 100
        self.last_speed = None      
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.direction_pin, GPIO.OUT)
    def set_acceleration(self, acceleration):
        self.acceleration = acceleration
    def set_max_velocity(self, max_velocity):
        self.max_velocity = max_velocity
    def set_deceleration(self, deceleration):
        self.deceleration = deceleration
    def move_steps(self, steps):
        direction = GPIO.HIGH if steps > 0 else GPIO.LOW
        GPIO.output(self.direction_pin, direction)
        steps_to_max_velocity = (self.max_velocity - self.current_velocity) / self.acceleration
        self.last_speed = self.current_velocity + abs(steps) * self.acceleration
        if steps > 0 and abs(steps) <= steps_to_max_velocity:
            self._move_with_acceleration(steps, direction)
        elif steps > 0 and abs(steps) > steps_to_max_velocity:
            self._move_with_acceleration(int(steps_to_max_velocity), direction)
            remaining_steps = abs(steps) - int(steps_to_max_velocity)
            self._move_with_constant_velocity(remaining_steps, direction)
        elif steps < 0:
            self._move_with_deceleration(steps, direction)
        else:
            print("move_steps: something is wrong with input steps.")
    def _move_with_acceleration(self, steps, direction):
        acceleration_rate = self.acceleration
        current_velocity = self.current_velocity
        for i in range(abs(steps)):
            delay = 1 / current_velocity
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)
            current_velocity += acceleration_rate
    def _move_with_constant_velocity(self, steps, direction):
        delay = 1 / self.max_velocity
        for i in range(steps):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)
    def _move_with_deceleration(self, steps, direction):
        deceleration = self.deceleration
        if self.last_speed is None:
            return 
        current_velocity = self.last_speed
        for i in range(abs(steps)):
            delay = 1 / current_velocity
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)
            current_velocity -= deceleration
class Preprocessing:
    def __init__(self, url, model_degree):
        self.url = url
        self.model_degree = model_degree
        self.dataFrame = pd.read_excel(url, header=0)
        self.features = np.array(self.dataFrame['Force']).reshape(-1, 1)
        self.labels = np.array(self.dataFrame.drop(['Force'], axis=1)).reshape(-1, 1)
        self.polynomial = sklearn.preprocessing.PolynomialFeatures(model_degree, include_bias=False)
        self.Polynomial_features = self.polynomial.fit_transform(self.features)
        self.scaler = sklearn.preprocessing.StandardScaler().fit(self.Polynomial_features)
    def poly_transform(self, input_data):
        return self.polynomial.transform(input_data)  
    def scale(self, input_data):
        return self.scaler.transform(input_data)
def calibrate_steps(steps):
    direction = GPIO.HIGH if steps > 0 else GPIO.LOW
    GPIO.output(driverDIR, direction)
    for _ in range(abs(steps)):
        GPIO.output(driverPUL, GPIO.HIGH)
        sleep(0.003)
        GPIO.output(driverPUL, GPIO.LOW)
        sleep(0.003)
ser = serial.Serial(
    port='/dev/ttyAMA10',
    baudrate=19200,
    timeout=1,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)
calibration = False
calibrated = False
calibrate_str = ser.readline().decode('utf-8', errors='ignore').strip()
if calibrate_str == 'Yes':
    GPIO.output(driverDIR, GPIO.HIGH)
    for i in range(10):
        GPIO.output(driverPUL, GPIO.HIGH)
        sleep(0.003)
        GPIO.output(driverPUL, GPIO.LOW)
        sleep(0.003)
    while not calibrated:
        raw_cmd = ser.read(2)
        if len(raw_cmd) < 2:
            continue
        move_one_step = int.from_bytes(raw_cmd, "little")
        if move_one_step == 1:
            calibrate_steps(1)
        elif move_one_step == 2:
            calibrate_steps(-1)
        elif move_one_step == 3:
            calibrate_steps(-200)
            calibrated = True
    calibration = True
elif calibrate_str == 'No':
    calibration = True
motor = StepperMotor(driverPUL, driverDIR)
experiment = False
while not experiment:
    raw_num_test = ser.read(2)
    if len(raw_num_test) < 2:
        continue
    num_Test = int.from_bytes(raw_num_test, "little")
    if num_Test != 0:
        experiment = True
iteration = False
while not iteration:
    raw_iter = ser.read(2)
    if len(raw_iter) < 2:
        continue
    num_iteration = int.from_bytes(raw_iter, "little")
    if num_iteration != 0:
        iteration = True
errorPercentage = False
while not errorPercentage:
    raw_err = ser.read(4)
    if len(raw_err) < 4:
        continue
    errorRate = struct.unpack('<f', raw_err)[0]
    if errorRate != 0:
        errorPercentage = True
model_received = False
model = None
preprocess = None
while not model_received:
    choice = ser.readline().decode('utf-8', errors='ignore').strip()
    if choice == 'black':
        model = joblib.load('FinalModel_blackTip.pkl')
        preprocess = Preprocessing('/home/wanjin/StepMotor/processed data_blackTip.xlsx', 2)
        model_received = True
    elif choice == 'plastic':
        model = joblib.load('FinalModel_plasticTip.pkl')
        preprocess = Preprocessing('/home/wanjin/StepMotor/processed data_PlasticTip.xlsx', 4)
        model_received = True
for _ in range(num_Test):
    error_acceptance_rate = False
    count = 1
    desiredF = False
    while not desiredF:
        dF_raw = ser.read(2)
        if len(dF_raw) < 2:
            continue
        dF = int.from_bytes(dF_raw, "little")
        if dF != 0:
            print(f"Desired Force = {dF}")
            inputs = np.array([[dF]], dtype=float)
            scaled_input = preprocess.scale(preprocess.poly_transform(inputs))
            prediction = int(model.predict(scaled_input).item())
            print(f"Model Prediction = {prediction}")       
            motor.last_speed = prediction
            acceleration = (prediction - motor.current_velocity) / Steps
            motor.set_acceleration(acceleration)
            motor.set_deceleration(acceleration)
            desiredF = True
    while not error_acceptance_rate:
        ser.write(b'1')
        sleep(2)
        motor.move_steps(Steps)
        motor.move_steps(-Steps)
        count += 1
        forceReceived = False
        while not forceReceived:
            newdata = ser.read(4)
            if len(newdata) == 4:
                fmax = struct.unpack('<f', newdata)[0]
                if fmax != 0:
                    forceReceived = True
                    print(f"Measured Force from MATLAB = {fmax:.2f}")
        error_ = (fmax - dF) / dF
        print(f"Error = {error_:.2%}")
        if abs(error_) <= (errorRate / 100.0):
            ser.write(b'1')
            error_acceptance_rate = True
        elif count >= num_iteration:
            ser.write(b'1')
            break
        else:
            if abs(error_) >= 1.0:
                acceleration = acceleration - (0.1 * error_ * acceleration)
            else:
                acceleration = acceleration - (error_ * acceleration)     
            motor.set_acceleration(acceleration)
            motor.set_deceleration(acceleration)
            sleep(2)
            ser.write(b'2')
print("All tests finished. Exiting...")
