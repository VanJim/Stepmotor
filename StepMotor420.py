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

print("StepMotor.py script started on Raspberry Pi...")

# GPIO pin definitions
driverPUL = 12
driverDIR = 18
Steps = 800  # Do not change this number of steps

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(driverPUL, GPIO.OUT)
GPIO.setup(driverDIR, GPIO.OUT)

class StepperMotor:
    def __init__(self, step_pin, direction_pin):
        self.step_pin = step_pin
        self.direction_pin = direction_pin
        self.acceleration = 10
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
            self._move_with_acceleration(steps, direction)
        elif abs(steps) > steps_to_max_velocity and steps > 0:
            self._move_with_acceleration(steps_to_max_velocity, direction)
            remaining_steps = steps - steps_to_max_velocity
            self._move_with_constant_velocity(remaining_steps, direction)
        elif steps < 0:
            self._move_with_deceleration(steps, direction)
        else:
            print("move_steps: Something might be wrong with the steps argument.")

    def _move_with_acceleration(self, steps, direction):
        GPIO.output(self.direction_pin, direction)
        acceleration_rate = self.acceleration
        current_velocity = self.current_velocity

        for _ in range(int(abs(steps))):
            delay = 1 / current_velocity
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)
            current_velocity += acceleration_rate

    def _move_with_constant_velocity(self, steps, direction):
        GPIO.output(self.direction_pin, direction)
        delay = 1 / self.max_velocity
        current_step = 0

        while current_step < steps:
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)
            current_step += 1

    def _move_with_deceleration(self, steps, direction):
        GPIO.output(self.direction_pin, direction)
        deceleration = self.deceleration
        if self.last_speed is None:
            return

        current_velocity = self.last_speed
        for _ in range(abs(steps)):
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

# Set up the serial port with a timeout
# 注意: /dev/ttyAMA10 如果不对，需要自行改成树莓派真实可用的 UART 端口
ser = serial.Serial('/dev/ttyAMA10', 115200, timeout=2)

calibration = False
calibrated = False
num_Test = 0
experiment = False
iteration = False
errorPercentage = False

MAX_TRIES = 10

#=========== Calibration phase ===========
tries = 0
while not calibration and tries < MAX_TRIES:
    data = ser.readline()
    if not data:
        print("[Calib] No data received or timeout, retrying...")
        tries += 1
        continue

    calibrate_str = data.decode('utf-8').strip()
    print(f"[Calib] Received: {calibrate_str}")

    if calibrate_str == 'Yes':
        GPIO.output(driverDIR, GPIO.HIGH)
        for _ in range(10):
            GPIO.output(driverPUL, GPIO.HIGH)
            sleep(0.003)
            GPIO.output(driverPUL, GPIO.LOW)
            sleep(0.003)

        while not calibrated:
            step_data = ser.readline()
            if not step_data:
                print("[Calib] No step instruction received or timeout. Breaking calibration.")
                break

            try:
                move_one_step = int.from_bytes(step_data, "little")
            except ValueError:
                print("[Calib] Unable to parse step instruction.")
                continue

            if move_one_step == 1:
                calibrate_steps(1)
            elif move_one_step == 2:
                calibrate_steps(-1)
            elif move_one_step == 3:
                calibrated = True
                calibration = True
                calibrate_steps(-200)
            else:
                print(f"[Calib] Unknown step instruction: {move_one_step}")

    elif calibrate_str == 'No':
        calibration = True
    else:
        print(f"[Calib] Unrecognized calibration command: {calibrate_str}")

    tries += 1

if not calibration:
    print("[Warning] Calibration phase did not complete successfully.")

motor = StepperMotor(driverPUL, driverDIR)

#=========== Receive number of tests ===========
tries = 0
while not experiment and tries < MAX_TRIES:
    data = ser.readline()
    if not data:
        print("[NumTest] No data or timeout, retrying...")
        tries += 1
        continue
    try:
        num_Test = int.from_bytes(data, "little")
        print(f"[NumTest] Received test count: {num_Test}")
        if num_Test != 0:
            experiment = True
    except ValueError:
        print("[NumTest] Unable to parse test count.")
    tries += 1

if not experiment:
    print("[Warning] Could not get a valid number of tests.")

#=========== Receive number of iterations ===========
tries = 0
while not iteration and tries < MAX_TRIES:
    data = ser.readline()
    if not data:
        print("[Iteration] No data or timeout, retrying...")
        tries += 1
        continue
    try:
        num_iteration = int.from_bytes(data, "little")
        print(f"[Iteration] Received iteration count: {num_iteration}")
        if num_iteration != 0:
            iteration = True
    except ValueError:
        print("[Iteration] Unable to parse iteration count.")
    tries += 1

if not iteration:
    print("[Warning] Could not get a valid number of iterations.")

#=========== Receive error rate ===========
tries = 0
errorRate = 0.0
while not errorPercentage and tries < MAX_TRIES:
    data = ser.read(4)
    if len(data) < 4:
        print("[ErrorRate] Did not receive 4 bytes or timeout, retrying...")
        tries += 1
        continue
    try:
        errorRate = struct.unpack('<f', data)[0]
        print(f"[ErrorRate] Received error rate: {errorRate}")
        if errorRate != 0:
            errorPercentage = True
    except struct.error:
        print("[ErrorRate] Struct unpack error.")
    tries += 1

if not errorPercentage:
    print("[Warning] Could not get a valid error rate.")

#=========== Model selection ===========
model_received = False
model = None
preprocess = None
tries = 0
while not model_received and tries < MAX_TRIES:
    data = ser.readline()
    if not data:
        print("[Model] No data or timeout, retrying...")
        tries += 1
        continue

    choice = data.decode('utf-8').strip()
    print(f"[Model] Received model choice: {choice}")

    if choice == 'black':
        model = joblib.load('FinalModel_blackTip.pkl')
        preprocess = Preprocessing('/home/wanjin/StepMotor/processed data_blackTip.xlsx', 2)
        model_received = True
    elif choice == 'plastic':
        model = joblib.load('FinalModel_plasticTip.pkl')
        preprocess = Preprocessing('/home/wanjin/StepMotor/processed data_PlasticTip.xlsx', 4)
        model_received = True
    else:
        print(f"[Model] Unrecognized model choice: {choice}")

    tries += 1

if not model_received:
    print("[Warning] Model choice was not received successfully.")

#=========== Main test process ===========
for test_idx in range(num_Test):
    error = 0
    acceleration = 0
    count = 1
    acceleration_values = []
    error_rates = []
    desiredF = False
    error_acceptance_rate = False

    tries = 0
    while not desiredF and tries < MAX_TRIES:
        data = ser.readline()
        if not data:
            print(f"[Test#{test_idx}] No desired force data or timeout, retrying...")
            tries += 1
            continue
        try:
            dF = int.from_bytes(data, "little")
            if dF != 0:
                print(f"[DesiredF] Target force: {dF}")
                inputs = np.array(dF).reshape(-1, 1)
                prediction = int(model.predict(preprocess.scale(preprocess.poly_transform(inputs))).item())
                print(f"[Prediction] Predicted speed: {prediction}")

                motor.last_speed = prediction
                acceleration = (prediction - motor.current_velocity) / Steps
                motor.set_acceleration(acceleration)
                motor.set_deceleration(acceleration)
                desiredF = True
        except ValueError:
            print(f"[Test#{test_idx}] Unable to parse desired force.")
        tries += 1

    if not desiredF:
        print(f"[Test#{test_idx}] No valid desired force, skipping this test.")
        continue

    while not error_acceptance_rate and count <= num_iteration:
        ser.write(b'1')
        sleep(2)

        motor.move_steps(Steps)
        motor.move_steps(-Steps)
        count += 1

        forceRecieved = False
        sub_tries = 0
        fmax = 0
        while not forceRecieved and sub_tries < MAX_TRIES:
            newdata = ser.read(4)
            if len(newdata) < 4:
                print("[Force] No 4-byte data for force or timeout, retrying...")
                sub_tries += 1
                continue
            try:
                fmax = struct.unpack('<f', newdata)[0]
                if fmax != 0:
                    print(f"[Force] Measured force: {fmax}")
                    forceRecieved = True
            except struct.error:
                print("[Force] Unable to unpack force data.")
            sub_tries += 1

        if not forceRecieved:
            print("[Force] No valid measured force, aborting this iteration.")
            break

        error = (fmax - dF) / dF
        print(f"[Test#{test_idx}] Current error: {error:.2%}")

        if abs(error) <= (errorRate / 100):
            error_acceptance_rate = True
            acceleration_values.append(acceleration)
            error_rates.append(abs(error))
            ser.write(b'1')
        else:
            acceleration_values.append(acceleration)
            error_rates.append(abs(error))
            if abs(error) >= 1:
                acceleration = acceleration - (0.1 * error * acceleration)
            else:
                acceleration = acceleration - (error * acceleration)
            motor.set_acceleration(acceleration)
            motor.set_deceleration(acceleration)
            sleep(2)
            ser.write(b'2')

        if count > num_iteration:
            ser.write(b'1')
            break

print("Process finished. Script ends here.")
# GPIO.cleanup()  # Uncomment if needed
