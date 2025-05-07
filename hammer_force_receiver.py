#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import serial
import sys
import logging

#—— 日志配置 ——#
LOG_FILE = '/home/wanjin/StepMotor/hammer_force_receiver.log'
logging.basicConfig(
    filename=LOG_FILE,
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(message)s'
)
console = logging.StreamHandler(sys.stdout)
console.setLevel(logging.INFO)
console.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
logging.getLogger().addHandler(console)

logging.info("=== Script starting ===")

#—— 步进电机引脚配置 ——#
STEP_PIN     = 12
DIR_PIN      = 18
STEPS_90_DEG = 50
SERIAL_PORT  = '/dev/serial0'
BAUDRATE     = 9600

#—— 初始化 GPIO ——#
try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(STEP_PIN, GPIO.OUT)
    GPIO.setup(DIR_PIN,  GPIO.OUT)
    logging.info("GPIO initialized")
except Exception as e:
    logging.error(f"GPIO init failed: {e}")
    sys.exit(1)

#—— 打开串口 ——#
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=None)
    logging.info(f"Serial port {SERIAL_PORT} opened at {BAUDRATE} baud")
except Exception as e:
    logging.error(f"Failed to open serial port {SERIAL_PORT}: {e}")
    sys.exit(1)

def strike(intensity):
    freq = max(1, intensity)
    half_period = 1.0 / freq / 2.0
    logging.info(f"Strike: intensity={intensity}, half_period={half_period:.6f}s")

    # 正向敲击
    GPIO.output(DIR_PIN, GPIO.HIGH)
    for _ in range(STEPS_90_DEG):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(half_period)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(half_period)

    time.sleep(0.2)

    # 反向回原位
    GPIO.output(DIR_PIN, GPIO.LOW)
    for _ in range(STEPS_90_DEG):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(half_period)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(half_period)

    logging.info("Strike completed")

try:
    while True:
        line = ser.readline().decode('ascii', 'ignore').strip()
        logging.info(f"Received raw line: '{line}'")
        if not line:
            continue
        try:
            inten = int(line)
            logging.info(f"Parsed intensity: {inten}")
            strike(inten)
        except ValueError:
            logging.warning(f"Ignoring non-integer input: '{line}'")
            continue
finally:
    ser.close()
    GPIO.cleanup()
    logging.info("=== Script exiting ===")
