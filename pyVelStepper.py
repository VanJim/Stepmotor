import time
import RPi.GPIO as GPIO

class StepperMotor:
    def __init__(self, step_pin, direction_pin):
        self.step_pin = step_pin
        self.direction_pin = direction_pin
        self.acceleration = 20
        self.max_velocity = 200000
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
