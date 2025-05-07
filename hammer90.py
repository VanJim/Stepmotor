#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import sys

# —— 配置引脚 —— #
STEP_PIN       = 12      # PUL
DIR_PIN        = 18      # DIR
STEPS_90_DEG   = 213     # 一圈 1000 步，90° 即 215 步
STEPS_20_DEG   = int(STEPS_90_DEG * 20 / 90)       # ≈ 48 步
STEPS_REMAIN   = STEPS_90_DEG - STEPS_20_DEG       # ≈ 167 步
DEFAULT_DELAY  = 0.001   # 默认脉冲间隔 (秒)

# —— 速度设置 —— #
FAST_DELAY = 0.0005      # 较快速度
SLOW_DELAY = 0.001       # 较慢速度

def step(steps, direction, delay):
    GPIO.output(DIR_PIN, direction)
    for _ in range(abs(steps)):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(delay)

def main():
       # 解析命令行参数：force（大于 0 的浮点数）
    force = 1.0
    if len(sys.argv) > 1:
        try:
            force = float(sys.argv[1])
            if force <= 0:
                raise ValueError
        except ValueError:
            print(f"错误：force 必须是大于 0 的数字，收到 '{sys.argv[1]}'")
            sys.exit(1)

    # 根据 force 计算实际 delay（force 越大，delay 越小，力度越强）
    delay = DEFAULT_DELAY / force
   
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(STEP_PIN, GPIO.OUT)
    GPIO.setup(DIR_PIN,  GPIO.OUT)

    try:
        # 1) 正向 90° 敲击
        step(STEPS_90_DEG, GPIO.HIGH, delay)

        # 2) 反向：先回 20°（快），再回剩余 70°（慢）
        step(STEPS_20_DEG, GPIO.LOW, FAST_DELAY)
        step(STEPS_REMAIN, GPIO.LOW, SLOW_DELAY)

    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
