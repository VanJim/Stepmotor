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

# GPIO pins
driverPUL = 12
driverDIR = 18
Steps = 800  # Do not change

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
        self.deceleration = 10
        self.current_velocity = 10
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
        
        if abs(steps) <= steps_to_max_velocity and steps > 0:
            self._move_with_acceleration(abs(steps), direction)
        elif abs(steps) > steps_to_max_velocity and steps > 0:
            self._move_with_acceleration(int(steps_to_max_velocity), direction)
            remaining = abs(steps) - int(steps_to_max_velocity)
            self._move_with_constant_velocity(remaining, direction)
        elif steps < 0:
            self._move_with_deceleration(abs(steps), direction)
        else:
            print("Warning: unexpected step input")

    def _move_with_acceleration(self, steps, direction):
        current_velocity = self.current_velocity
        for _ in range(steps):
            delay = 1 / current_velocity
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)
            current_velocity += self.acceleration

    def _move_with_constant_velocity(self, steps, direction):
        delay = 1 / self.max_velocity
        for _ in range(steps):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)

    def _move_with_deceleration(self, steps, direction):
        if self.last_speed is None:
            return
        current_velocity = self.last_speed
        for _ in range(steps):
            delay = 1 / current_velocity
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)
            current_velocity -= self.deceleration


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


# Serial port at 9600 baudrate
ser = serial.Serial('/dev/ttyAMA1', 9600, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

calibration = False
calibrated = False
experiment = False
iteration = False
errorPercentage = False

motor = StepperMotor(driverPUL, driverDIR)

# Calibration
while not calibration:
    # Read "Yes"/"No" (text line)
    calibrate_resp = ser.readline().decode('utf-8').strip()
    if calibrate_resp == 'Yes':
        GPIO.output(driverDIR, GPIO.HIGH)
        for _ in range(10):
            GPIO.output(driverPUL, GPIO.HIGH)
            sleep(0.003)
            GPIO.output(driverPUL, GPIO.LOW)
            sleep(0.003)

        while not calibrated:
            raw_move = ser.read(2)
            if len(raw_move) == 2:
                move_one_step = int.from_bytes(raw_move, "little")
                if move_one_step == 1:
                    calibrate_steps(1)
                elif move_one_step == 2:
                    calibrate_steps(-1)
                elif move_one_step == 3:
                    calibrate_steps(-200)
                    calibrated = True
                    calibration = True
    elif calibrate_resp == 'No':
        calibration = True

# Number of tests
num_Test = 0
while not experiment:
    raw_test = ser.read(2)
    if len(raw_test) == 2:
        num_Test = int.from_bytes(raw_test, "little")
        if num_Test != 0:
            experiment = True

# Number of iterations
num_iteration = 0
while not iteration:
    raw_iter = ser.read(2)
    if len(raw_iter) == 2:
        num_iteration = int.from_bytes(raw_iter, "little")
        if num_iteration != 0:
            iteration = True

# Error rate (4 bytes float)
errorRate_val = 0.0
while not errorPercentage:
    raw_err = ser.read(4)
    if len(raw_err) == 4:
        errorRate_val = struct.unpack('<f', raw_err)[0]
        if errorRate_val != 0:
            errorPercentage = True

# Model selection
model_received = False
model = None
preprocess = None

while not model_received:
    choice = ser.readline().decode('utf-8').strip()
    if choice == 'black':
        model = joblib.load('FinalModel_blackTip.pkl')
        preprocess = Preprocessing(r'/home/labraspberry/StepMotor/processed data_blackTip.xlsx', 2)
        model_received = True
    elif choice == 'plastic':
        model = joblib.load('FinalModel_plasticTip.pkl')
        preprocess = Preprocessing(r'/home/labraspberry/StepMotor/processed data_PlasticTip.xlsx', 4)
        model_received = True

# Main test loop
for _ in range(num_Test):
    error_val = 0.0
    acceleration = 0.0
    count = 1
    acceleration_values = []
    error_rates = []
    desiredF = False
    error_acceptance_rate = False

    # Read desired force (2 bytes uint16)
    dF = 0
    while not desiredF:
        raw_dF = ser.read(2)
        if len(raw_dF) == 2:
            dF = int.from_bytes(raw_dF, "little")
            if dF != 0:
                print("Desired Force:", dF)
                inputs = np.array(dF).reshape(-1, 1)
                prediction = int(model.predict(preprocess.scale(preprocess.poly_transform(inputs))).item())
                print("Initial speed prediction:", prediction)
                motor.last_speed = prediction
                acceleration = (prediction - motor.current_velocity) / Steps
                motor.set_acceleration(acceleration)
                motor.set_deceleration(acceleration)
                desiredF = True

    while not error_acceptance_rate:
        # Trigger DAQ in MATLAB by sending ASCII '1'
        ser.write(b'1')
        sleep(2)

        # Move motor
        motor.move_steps(Steps)
        motor.move_steps(-Steps)
        count += 1

        # Read back measured force (4 bytes float)
        forceRecieved = False
        fmax = 0.0
        while not forceRecieved:
            newdata = ser.read(4)
            if len(newdata) == 4:
                fmax = struct.unpack('<f', newdata)[0]
                if fmax != 0:
                    forceRecieved = True
                    print("Measured Force:", fmax)

        # Calculate error
        error_val = (fmax - dF) / dF
        if abs(error_val) <= (errorRate_val / 100.0):
            ser.write(b'1')  # Acceptable error
            error_acceptance_rate = True
            acceleration_values.append(acceleration)
            error_rates.append(abs(error_val))
        elif count >= num_iteration:
            ser.write(b'1')  # Stop if iteration limit reached
            break
        else:
            # Adjust acceleration
            acceleration_values.append(acceleration)
            error_rates.append(abs(error_val))
            if abs(error_val) >= 1:
                acceleration = acceleration - (0.1 * error_val * acceleration)
            else:
                acceleration = acceleration - (error_val * acceleration)
            motor.set_acceleration(acceleration)
            motor.set_deceleration(acceleration)
            sleep(2)
            ser.write(b'2')  # Not acceptable yet
