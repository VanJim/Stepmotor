import RPi.GPIO as GPIO
import time

driverPUL = 12  
driverDIR = 18  

Steps_per_revolution = 795

class StepperMotor:
    def __init__(self, step_pin, direction_pin):
        self.step_pin = step_pin
        self.direction_pin = direction_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.direction_pin, GPIO.OUT)

    def move_steps_constant_speed(self, steps, speed):
   
        direction = GPIO.HIGH if steps > 0 else GPIO.LOW
        GPIO.output(self.direction_pin, direction)

        step_delay = 1.0 / (2.0 * abs(speed))  
        for _ in range(abs(steps)):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(step_delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(step_delay)

    def move_steps_linear_decel(self, steps, start_speed, end_speed):
  
        direction = GPIO.HIGH if steps > 0 else GPIO.LOW
        GPIO.output(self.direction_pin, direction)

        total_steps = abs(steps)
       
        if total_steps <= 1:
            avg_speed = (start_speed + end_speed) / 2.0
            self.move_steps_constant_speed(steps, avg_speed)
            return

        for i in range(total_steps):
            # 线性插值：i 从 0 到 total_steps-1
            ratio = i / float(total_steps - 1)  
            # 当前速度从 start_speed 过渡到 end_speed
            current_speed = start_speed + (end_speed - start_speed) * ratio

            # 注意：避免 current_speed = 0
            if current_speed <= 0:
                current_speed = 1  

            half_pulse_delay = 1.0 / (2.0 * current_speed)

            # 发出一个脉冲
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(half_pulse_delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(half_pulse_delay)
if __name__ == "__main__":
    try:
        motor = StepperMotor(step_pin=driverPUL, direction_pin=driverDIR)
        
        # 可以根据实际需求调整的几个参数：
        forward_speed = 145           # 正转的恒定速度（steps/s）
        reverse_start_speed = 300     # 反转开始时的较快速度
        reverse_end_speed = 100        # 反转结束时的较慢速度

        # 演示：转 90 度所需步数
        steps_for_90_deg = int((90.0 / 360.0) * Steps_per_revolution)

        print("Motor rotating forward (constant speed)...")
        motor.move_steps_constant_speed(steps_for_90_deg, speed=forward_speed)

        # 不等待，立刻反转
        print("Motor reversing (fast -> slow)...")
        motor.move_steps_linear_decel(-steps_for_90_deg,
                                      start_speed=reverse_start_speed,
                                      end_speed=reverse_end_speed)
        print("Done.")
        GPIO.cleanup()

    except KeyboardInterrupt:
        GPIO.cleanup()
