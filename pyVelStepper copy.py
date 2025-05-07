class StepperMotor:
    def __init__(self, step_pin, direction_pin):
        self.step_pin = step_pin
        self.direction_pin = direction_pin
        self.acceleration = 10 # This is the rate of change in PWM signal frequency, therefore the Unit is [1/s or (Step/s)/Step]
        self.max_velocity = 100000  # Pulse per second or [step/s]
        self.deceleration = 10 # This is the rate of change in PWM signal frequency, therefore the Unit is [1/s or (Step/s)/Step]
        self.current_velocity = 10 # Change based on your motor
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
