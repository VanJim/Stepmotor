import numpy as np 
import pandas as pd
import joblib
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
        self.max_velocity = 200000
        self.deceleration = 20
        self.current_velocity = 100
        self.last_speed = None

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
            remain = abs(steps) - steps_to_max_velocity
            self._move_with_constant_velocity(remain, direction)
        elif steps < 0:
            self._move_with_deceleration(steps, direction)
        else:
            print("Something's wrong with steps input.")
            return

    def _move_with_acceleration(self, steps, direction):
        acceleration_rate = self.acceleration
        current_velocity = self.current_velocity
        for i in range(int(abs(steps))):
            delay = 1.0 / current_velocity
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)
            current_velocity += acceleration_rate

    def _move_with_constant_velocity(self, steps, direction):
        delay = 1.0 / self.max_velocity
        for i in range(int(steps)):
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
            delay = 1.0 / current_velocity
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)
            current_velocity -= deceleration

def calibrate_steps(steps):
    direction = GPIO.HIGH if steps > 0 else GPIO.LOW
    GPIO.output(driverDIR, direction)
    for _ in range(abs(steps)):
        GPIO.output(driverPUL, GPIO.HIGH)
        time.sleep(0.003)
        GPIO.output(driverPUL, GPIO.LOW)
        time.sleep(0.003)

class Preprocessing:
    def __init__(self, url, model_degree):
        self.dataFrame = pd.read_excel(url, header=0)
        self.features = np.array(self.dataFrame['Force']).reshape(-1, 1)
        self.labels = np.array(self.dataFrame.drop(['Force'], axis=1)).reshape(-1, 1)
        from sklearn.preprocessing import PolynomialFeatures, StandardScaler
        self.polynomial = PolynomialFeatures(model_degree, include_bias=False)
        self.Polynomial_features = self.polynomial.fit_transform(self.features)
        self.scaler = StandardScaler().fit(self.Polynomial_features)
       
    def poly_transform(self, input_data):
        return self.polynomial.transform(input_data)
        
    def scale(self,input_data):
        return self.scaler.transform(input_data)

ser = serial.Serial('/dev/ttyAMA10',115200, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

calibration = False
calibrated = False
num_Test = 0
experiment = False
iteration = False
errorPercentage = False

while not calibration:
    calibrate_data = ser.readline().decode('utf-8').strip()
    if calibrate_data == 'Yes':
        GPIO.output(driverDIR, GPIO.HIGH)
        for i in range(10):
            GPIO.output(driverPUL, GPIO.HIGH)
            time.sleep(0.003)
            GPIO.output(driverPUL, GPIO.LOW)
            time.sleep(0.003)

        while not calibrated:
            step_cmd = ser.readline()
            move_one_step = int.from_bytes(step_cmd, "little") if len(step_cmd) > 0 else 0
            if move_one_step == 1:
                calibrate_steps(1)
            elif move_one_step == 2:
                calibrate_steps(-1)
            elif move_one_step == 3:
                calibrate_steps(-200) 
                calibrated = True
                calibration = True

    elif calibrate_data == 'No':
        calibration = True

motor = StepperMotor(driverPUL, driverDIR)
while not experiment:
    numTest_raw = ser.readline()
    if len(numTest_raw) > 0:
        num_Test = int.from_bytes(numTest_raw, "little")
        if num_Test != 0:
            experiment = True

while not iteration:
    iteration_raw = ser.readline()
    if len(iteration_raw) > 0:
        num_iteration = int.from_bytes(iteration_raw, "little")
        if num_iteration != 0:
            iteration = True

while not errorPercentage:
    errorRate_raw = ser.read(4)
    if len(errorRate_raw) == 4:
        errorRate = struct.unpack('<f', errorRate_raw)[0]
        if errorRate != 0:
            errorPercentage = True

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

for _ in range(num_Test):
    error = 0
    acceleration = 0
    count = 1
    desiredF = False
    error_acceptance_rate = False

    while not desiredF:
        data = ser.readline()
        if len(data) > 0:
            dF = int.from_bytes(data, "little")
            if dF != 0:
                print("Desired Force:", dF)
                inputs = np.array(dF).reshape(-1,1)
                prediction = int(model.predict(preprocess.scale(preprocess.poly_transform(inputs))).item())
                print("Prediction (accel):", prediction)
                
                motor.last_speed = prediction
                acceleration = (prediction - motor.current_velocity) / Steps
                motor.set_acceleration(acceleration)
                motor.set_deceleration(acceleration)
                desiredF = True

    while not error_acceptance_rate:
        ser.write(b'1')
        time.sleep(2)
        motor.move_steps(Steps)
        motor.move_steps(-Steps)
        count += 1

        forceReceived = False
        while not forceReceived:
            newdata = ser.read(4)
            if len(newdata) == 4:
                fmax = struct.unpack('<f', newdata)[0]
                if fmax != 0:
                    print("Measured Force:", fmax)
                    forceReceived = True

        error = (fmax - dF) / dF
        print("Error:", error)

        if abs(error) <= (errorRate / 100):
            error_acceptance_rate = True
            ser.write(b'1')
        elif count >= num_iteration:
            ser.write(b'1')
            break
        else:
            if abs(error) >= 1.0:
                acceleration = acceleration - (0.1 * error * acceleration)
            else:
                acceleration = acceleration - (error * acceleration)
            motor.set_acceleration(acceleration)
            motor.set_deceleration(acceleration)
            time.sleep(2)
            ser.write(b'2') 

print("All tests completed.")
GPIO.cleanup()
