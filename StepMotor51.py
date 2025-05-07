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

# ------------------------------
# Hardware setup
# ------------------------------
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
        self.acceleration = 10         # [steps/s per step]
        self.max_velocity = 100000     # [steps/s]
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
        self.last_speed = self.current_velocity + abs(steps)*self.acceleration

        if abs(steps) <= steps_to_max_velocity and steps > 0:
            self._move_with_acceleration(steps, direction)
        elif abs(steps) > steps_to_max_velocity and steps > 0:
            self._move_with_acceleration(steps_to_max_velocity, direction)
            remain_steps = abs(steps) - steps_to_max_velocity
            self._move_with_constant_velocity(remain_steps, direction)
        elif steps < 0:
            self._move_with_deceleration(steps, direction)
        else:
            print("Invalid step input.")
            return

    def _move_with_acceleration(self, steps, direction):
        steps = int(steps)
        current_velocity = self.current_velocity
        for _ in range(abs(steps)):
            delay = 1 / current_velocity
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)
            current_velocity += self.acceleration

    def _move_with_constant_velocity(self, steps, direction):
        steps = int(steps)
        delay = 1 / self.max_velocity
        for _ in range(steps):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)

    def _move_with_deceleration(self, steps, direction):
        steps = abs(steps)
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

# ------------------------------
# Serial initialization
# ------------------------------
ser = serial.Serial('/dev/ttyAMA0', 9600, timeout= 5)

calibration = False
calibrated = False
num_Test = 0
experiment = False
iteration = False
errorPercentage = False

# ------------------------------
# Calibration stage
# ------------------------------
while not calibration:
    # Expecting "Yes\n" or "No\n" from MATLAB
    calibrate = ser.readline().decode('utf-8').strip()
    if calibrate == 'Yes':
        GPIO.output(driverDIR, GPIO.HIGH)
        for _ in range(10):
            GPIO.output(driverPUL, GPIO.HIGH)
            sleep(0.003)
            GPIO.output(driverPUL, GPIO.LOW)
            sleep(0.003)

        while not calibrated:
            # Expecting 2 bytes: 1, 2, or 3 from MATLAB
            move_bytes = ser.read(2)
            if len(move_bytes) < 2:
                continue
            move_cmd = int.from_bytes(move_bytes, "little")
            if move_cmd == 1:    # move forward by 1 step
                calibrate_steps(1)
            elif move_cmd == 2: # move backward by 1 step
                calibrate_steps(-1)
            elif move_cmd == 3: # move backward 200 steps
                calibrate_steps(-200)
                calibrated = True
                calibration = True

    elif calibrate == 'No':
        calibration = True

# ------------------------------
# Receive num_Test (2 bytes)
# ------------------------------
motor = StepperMotor(driverPUL, driverDIR)
while not experiment:
    test_bytes = ser.read(2)
    if len(test_bytes) < 2:
        continue
    num_Test = int.from_bytes(test_bytes, "little")
    if num_Test != 0:
        experiment = True

# ------------------------------
# Receive num_iteration (2 bytes)
# ------------------------------
while not iteration:
    it_bytes = ser.read(2)
    if len(it_bytes) < 2:
        continue
    num_iteration = int.from_bytes(it_bytes, "little")
    if num_iteration != 0:
        iteration = True

# ------------------------------
# Receive errorRate (4 bytes, float)
# ------------------------------
while not errorPercentage:
    errorRateBytes = ser.read(4)
    if len(errorRateBytes) == 4:
        errorRate = struct.unpack('<f', errorRateBytes)[0]
        print("Error Rate:", errorRate)
        if abs(errorRate) > 1e-9:
            errorPercentage = True

# ------------------------------
# Receive hammer tip selection (text line)
# ------------------------------
model_received = False
model = None
preprocess = None

while not model_received:
    choice = ser.readline().decode('utf-8').strip()
    if choice == 'black':
        model = joblib.load('FinalModel_blackTip.pkl')
        preprocess = Preprocessing('/home/wanjin/StepMotor/processed data_blackTip.xlsx', 2)
        model_received = True
    elif choice == 'plastic':
        model = joblib.load('FinalModel_plasticTip.pkl')
        preprocess = Preprocessing('/home/wanjin/StepMotor/processed data_PlasticTip.xlsx', 4)
        model_received = True

# ------------------------------
# Main test loop
# ------------------------------
for _ in range(num_Test):
    error = 0
    acceleration = 0
    count = 1
    acceleration_values = []
    error_rates = []
    desiredF = False
    error_acceptance_rate = False

    # --------------------------
    # 1) Receive desired force (2 bytes)
    # --------------------------
    while not desiredF:
        df_bytes = ser.read(2)
        if len(df_bytes) < 2:
            continue
        dF = int.from_bytes(df_bytes, "little")
        if dF != 0:
            print("Desired force:", dF)
            force_input = np.array(dF).reshape(-1,1)
            prediction = int(model.predict(preprocess.scale(preprocess.poly_transform(force_input))).item())
            print("Predicted velocity:", prediction)
            motor.last_speed = prediction
            acceleration = (prediction - motor.current_velocity) / Steps
            motor.set_acceleration(acceleration)
            motor.set_deceleration(acceleration)
            desiredF = True

    # --------------------------
    # 2) Iteration to achieve acceptable error
    # --------------------------
    while not error_acceptance_rate:
        # Send single byte to MATLAB to trigger data acquisition
        ser.write(b'1')
        sleep(2)

        # Move forward and backward
        motor.move_steps(Steps)
        motor.move_steps(-Steps)
        count += 1

        # Receive max force from MATLAB (4 bytes float)
        forceReceived = False
        fmax = 0.0
        while not forceReceived:
            fmaxBytes = ser.read(4)
            if len(fmaxBytes) == 4:
                fmax = struct.unpack('<f', fmaxBytes)[0]
                print("Fmax =", fmax)
                if abs(fmax) > 1e-6:
                    forceReceived = True

        error = (fmax - dF) / dF
        print("Error =", error)

        # If error is within the acceptable threshold
        if abs(error) <= (errorRate / 100.0):
            error_acceptance_rate = True
            acceleration_values.append(acceleration)
            error_rates.append(abs(error))
            ser.write(b'1')  # Signal success to MATLAB

        # If reached max iteration
        elif count >= num_iteration:
            ser.write(b'1')
            break

        # Otherwise, adjust acceleration and continue
        else:
            acceleration_values.append(acceleration)
            error_rates.append(abs(error))
            if abs(error) >= 1.0:
                acceleration = acceleration - (0.1 * error * acceleration)
            else:
                acceleration = acceleration - (error * acceleration)
            motor.set_acceleration(acceleration)
            motor.set_deceleration(acceleration)
            sleep(2)
            ser.write(b'2')  # Signal MATLAB to try another iteration

# ------------------------------
# Optional repeatability tests or other features
# ------------------------------
# You can add extra logic here if needed.
# ------------------------------

