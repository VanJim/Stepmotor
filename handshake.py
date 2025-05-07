#!/usr/bin/env python3
# handshake.py
import serial, time, sys

# serial0 是对 /dev/ttyAMA0 或 /dev/ttyS0 的映射
# 在树莓派新版本中通常 /dev/serial0 会链接到硬件 UART
PORT = '/dev/serial0'
BAUD = 9600

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(1)  # 等它稳定
except Exception as e:
    print(f"ERROR: 打开 {PORT} 失败: {e}", file=sys.stderr)
    sys.exit(1)

# 连发 10 条
for i in range(10):
    msg = f"HANDSHAKE {i+1}\n".encode('ascii')
    ser.write(msg)
    ser.flush()
    time.sleep(1)

ser.close()
