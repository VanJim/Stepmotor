#import required libraries
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
#variable decleration.Change this based on your system
driverPUL=12
driverDIR=18
Steps = 200 # Do not change number of steps

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(driverPUL, GPIO.OUT)
GPIO.setup(driverDIR, GPIO.OUT)

class StepperMotor:
    def __init__(self, step_pin, direction_pin):
        self.step_pin = step_pin
        self.direction_pin = direction_pin
        self.acceleration = 20 # This is the rate of change in PWM signal frequency, therefore the Unit is [1/s or (Step/s)/Step]
        self.max_velocity = 200000  # Pulse per second or [step/s]
        self.deceleration = 20 # This is the rate of change in PWM signal frequency, therefore the Unit is [1/s or (Step/s)/Step]
        self.current_velocity = 100 # Change based on your motor
        self. last_speed = None

        # Initialize GPIO
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

        # Acceleration phase
        steps_to_max_velocity =(self.max_velocity- self.current_velocity)/self.acceleration #[checking units: it is pulse or step ]
        self.last_speed = self.current_velocity + abs(steps)*self.acceleration
        
        if abs(steps) <= steps_to_max_velocity and steps>0:
            self._move_with_acceleration(steps, direction)
            
        elif abs(steps)> steps_to_max_velocity and steps>0 :
            # Acceleration phase
            self._move_with_acceleration(steps_to_max_velocity, direction)
            
            # Constant velocity phase
            steps = steps - (steps_to_max_velocity) #remianing_steps
            self._move_with_constant_velocity(steps, direction)
        elif steps<0:
        # Deceleration phase
            self._move_with_deceleration(steps, direction)
        else:
            print("Sorry, something is being passed Wrong")
            return
    
    def _move_with_acceleration(self, steps, direction):
        GPIO.output(self.direction_pin, direction)
        acceleration_rate = self.acceleration
        current_velocity = self.current_velocity
        
        for i in range(int(abs(steps))):
            delay = 1/current_velocity #provides the time duration for each step. So, the units for delay would be seconds per step (s/step)
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)
            current_velocity += acceleration_rate
        return
    
    def _move_with_constant_velocity(self, steps, direction):
        GPIO.output(self.direction_pin, direction)
        delay = 1/self.max_velocity
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
            # No previous acceleration phase
            return

        current_velocity = self.last_speed
        
        #deceleration phase
        for i in range(abs(steps)):
            delay = 1/current_velocity
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)
            current_velocity -= deceleration

#Preprocessing Class: Prepares input for model predictions
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
        
    def scale(self,input_data):
        return self.scaler.transform(input_data)
    

def calibrate_steps(steps):
    direction = GPIO.HIGH if steps > 0 else GPIO.LOW
    GPIO.output(driverDIR,direction)
    for _ in range(abs(steps)):
        GPIO.output(driverPUL, GPIO.HIGH)
        sleep(0.003)
        GPIO.output(driverPUL, GPIO.LOW)
        sleep(0.003)
        
# Establish serial communication   
ser=serial.Serial('/dev/ttyAMA0', 9600, timeout=5, parity=serial.PARITY_NONE,
                  stopbits=serial.STOPBITS_ONE)

#Do not change these variables
calibration = False
calibrated = False
num_Test = 0
experiment = False
iteration = False
errorPercentage = False
#calibration process
while not calibration:
    calibrate = ser.readline().decode('utf-8').strip()
    if calibrate == 'Yes':
        GPIO.output(driverDIR,GPIO.HIGH) #You may not need this first part. This is just for initializing motor movement
        for i in range(10):
            GPIO.output(driverPUL, GPIO.HIGH)
            sleep(0.003)
            GPIO.output(driverPUL, GPIO.LOW)
            sleep(0.003)

        while not calibrated:
            move_one_step = ser.read(2)
            if len(move_bytes) < 2:
                continue
            move_cmd = int.from_bytes(move_bytes, "little")
            
            if move_cmd == 1:#Move one step forward
               calibrate_steps(1)

            elif move_cmd == 2:#Move one step backward  
               calibrate_steps(-1)

            elif move_cmd == 3:#Come back 200 steps
                
                calibrate_steps(-200)
                calibrated = True
                calibration = True
                
    elif calibrate == 'No':
        calibration = True

#Receive Number of Tests
motor = StepperMotor(driverPUL,driverDIR)
while not experiment:
    test_bytes = ser.read(2)
    if len(test_bytes) < 2:
        continue
    num_Test = int.from_bytes(test_bytes,"little")
    if num_Test !=0:
        experiment = True
        
#Receive Number of Iteration
while not iteration:
    it_bytes = ser.read(2)
    if len(it_bytes) < 2:
        continue
    num_iteration = int.from_bytes(it_bytes,"little")
    if num_iteration !=0:
        iteration = True
    
#Receive Error_rate
while not errorPercentage:
    errorRate = ser.read(4)
    if len(errorRateBytes) == 4:
        errorRate = struct.unpack('<f', errorRateBytes)[0]
        print("Eroor Rate:", errorRate)
        if abs(errorRate) > 1e-9:
            errorPercentage = True

#Model selection part: Based on the hammer tip
model_received = False
model = None
preprocess = None

while not model_received:
    choice = ser.readline().decode('utf-8').strip()   # Assuming 'ser' is a valid serial port object

    if choice == 'black':
        model = joblib.load('FinalModel_blackTip.pkl')
        preprocess = Preprocessing(r'/home/wanjin/StepMotor/processed data_blackTip.xlsx', 2)
        model_received = True

    elif choice == 'plastic':
        model = joblib.load('FinalModel_plasticTip.pkl')
        preprocess = Preprocessing(r'/home/wanjin/StepMotor/processed data_PlasticTip.xlsx', 4)
        model_received = True

#Run Tests:
for _ in range(num_Test):
    error = 0
    acceleration = 0
    count = 1
    acceleration_values = []
    error_rates = []
    desiredF=False
    error_acceptance_rate = False

    while not desiredF:
        df_bytes = ser.read(2)
        if len(df_bytes) < 2:
            continue
        dF=int.from_bytes(df_bytes, "little")
        if dF !=0:
            print("Desired force:", dF)
            force_input = np.array(dF).reshape(-1, 1)
            prediction = int(model.predict(preprocess.scale(preprocess.poly_transform(force_input))).item())
            print("Predicted velocity:", prediction)
            motor.last_speed = prediction
            acceleration = (prediction - motor.current_velocity)/Steps
            motor.set_acceleration(acceleration)
            motor.set_deceleration(acceleration)
            desiredF=True
                
    while not error_acceptance_rate:
        #cdaqTrig:
        ser.write(b'1')
        sleep(2)
        motor.move_steps(Steps)
        motor.move_steps(-Steps)
        count +=1
        forceRecieved=False 
        fmax = 0.0  
        while not forceRecieved:
            fmaxBytes = ser.read(4)
            if len(fmaxBytes) == 4:
                fmax=struct.unpack('<f', fmaxBytes)[0]
                print("Fmax:", fmax)
                if abs(fmax) > 1e-6:
                   forceRecieved=True
        
        error = (fmax - dF)/dF
        print("Eroor =", error)
        
        if abs(error) <= (errorRate/100):
            error_acceptance_rate = True
            acceleration_values.append(acceleration)
            error_rates.append(abs(error))
            ser.write(b'1')
                    
        elif count>= num_iteration:
            ser.write(b'1')
            break
        elif abs(error) >=1:
            acceleration_values.append(acceleration)
            error_rates.append(abs(error))
            acceleration = acceleration - (0.1 *error * acceleration)
            motor.set_acceleration(acceleration)
            motor.set_deceleration(acceleration)
            sleep(2)
            ser.write(b'2')
        else:
            acceleration_values.append(acceleration)
            error_rates.append(abs(error))
            acceleration = acceleration - (error * acceleration)
            motor.set_acceleration(acceleration)
            motor.set_deceleration(acceleration)
            sleep(2)
            ser.write(b'2')
      
#Repeatability Test part: This is not part of the system. This is just for testing repeatability. Comment or delete.
    # repeatability= False
    # moto.set_acceleration(best_acceleration)
    # motor.set_deceleration(best_acceleration)
    # while not repeatability:
    #     repeat = ser.readline().decode('utf-8').strip()
    #     if repeat == 'Yes':
    #         for _ in range(15):
    #             #cdaqTrig
    #             ser.write(b'1')
    #             sleep(2)
    #             motor.move_steps(Steps)
    #             motor.move_steps(-Steps)
    #             forceRecieved = False
    #             while not forceRecieved:
    #                 fmax = 0
    #                 newdata = ser.readline()
    #                 fmax = int.from_bytes(newdata,"little")
    #                 if fmax !=0:
    #                     print(fmax)
    #                     forceRecieved = True
    #         repeatability= True
    #         ser.write(b'2')
    #         repeat = None
            

        



