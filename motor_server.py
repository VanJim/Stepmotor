import RPi.GPIO as GPIO
import time
import socket
import numpy as np
import numpy.polynomial.polynomial as poly

###############################################################################
# 1. 假设你之前做好的多项式拟合部分
###############################################################################
#    - excitations_data, speeds_data: 你的 500 组 (激励, speed) 数据
#    - 这里仅写个简单示例，请替换成你真实的数据与多项式系数
###############################################################################
excitations_data = np.array([
    0,3,3,3,3,4,4,5,5,5,
    6,6,7,8,8,8,9,9,9,9,
    9,10,10,10,10,11,11,13,12,14,
    14,16,17,16,16,17,17,17,18,18,
    18,19,20,20,21,21,22,22,24,24,
    25,27,28,31,31,33,36,41,47,48,
    42,46,43,37,30,33,32,31,31,32,
    33,34,36,35,36,37,38,39,40,41,
    40,41,44,45,45,46,46,47,48,48,
    50,52,55,50,53,55,60,57,62,63,
    62,60,66,68,70,73,65,64,68,70,
    76,78,70,76,76,75,78,80,82,90,
    88,88,92,91,92,93,96,97,101,100,
    100,102,104,105,106,107,112,114,112,114,
    115,116,118,121,121,119,120,120,122,123,
    125,125,116,128,128,128,128,132,132,133,
    135,142,138,140,138,140,142,145,146,148,
    146,149,153,162,158,155,162,160,163,160,
    161,165,168,169,170,172,172,178,174,170,
    172,176,179,182,181,180,183,185,186,188,
    188,192,190,192,193,199,201,202,205,206,
    211,205,207,212,210,212,215,213,210,218,
    210,215,215,220,220,223,220,220,223,225,
    228,227,229,231,231,231,228,232,242,240,
    238,243,248,242,246,248,246,251,249,250,
    252,252,251,252,250,260,252,262,263,259,
    262,263,263,260,271,270,263,265,268,273,
    275,278,272,275,274,276,268,285,281,288,
    293,286,288,295,292,296,296,298,294,298,
    298,310,305,306,308,300,308,313,315,308,
    312,306,311,306,313,320,312,303,318,306,
    310,315,312,318,325,316,310,322,342,340,
    338,343,350,365,338,340,343,358,340,360,
    362,352,361,358,363,380,398,355,358,378,
    375,395,385,418,415,400,380,385,380,398,
    383,402,390,390,410,399,405,410,406,410,
    418,412,408,402,425,408,412,430,418,406,
    408,415,420,418,420,435,422,418,395,410,
    419,406,418,426,438,440,416,422,430,450,
    438,445,430,430,436,452,432,443,437,435,
    440,442,456,450,436,444,443,422,428,436,
    424,422,439,422,455,434,432,418,416,432,
    455,426,444,430,436,446,442,442,438,456,
    436,442,436,442,438,442,472,478,470,472,
    470,448,482,485,490,495,510,500,502,492,
    525,490,492,499,490,498,496,530,506,499,
    500,498,505,506,510,510,502,500,515,503,
    522,512,516,526,534,520,530,531,518,532,
    528,518,538,526,519,518,516,520,522,540,
    516,522,524,528,522,524,539,522,542,536
    ])
speeds_data = np.array([
    0,5,10,15,20,25,30,35,40,45,
    50,55,60,65,70,75,80,85,90,95,
    100,105,110,115,120,125,130,135,140,145,
    150,155,160,165,170,175,180,185,190,195,
    200,205,210,215,220,225,230,235,240,245,
    250,255,260,265,270,275,280,285,290,295,
    300,305,310,315,320,325,330,335,340,345,
    350,355,360,365,370,375,380,385,390,395,400,405,410,
    415,420,425,430,435,440,445,450,455,460,
    465,470,475,480,485,490,495,500,505,510,
    515,520,525,530,535,540,545,550,555,560,
    565,570,575,580,585,590,595,600,605,610,
    615,620,625,630,635,640,645,650,655,660,
    665,670,675,680,685,690,695,700,705,710,
    715,720,725,730,735,740,745,750,755,760,    
    765,770,775,780,785,790,795,800,805,810,
    815,820,825,830,835,840,845,850,855,860,
    865,870,875,880,885,890,895,900,905,910,
    915,920,925,930,935,940,945,950,955,960,
    965,970,975,980,985,990,995,1000,1005,1010,
    1015,1020,1025,1030,1035,1040,1045,1050,1055,1060,
    1065,1070,1075,1080,1085,1090,1095,1100,1105,1110,
    1115,1120,1125,1130,1135,1140,1145,1150,1155,1160,
    1165,1170,1175,1180,1185,1190,1195,1200,1205,1210,
    1215,1220,1225,1230,1235,1240,1245,1250,1255,1260,
    1265,1270,1275,1280,1285,1290,1295,1300,1305,1310,
    1315,1320,1325,1330,1335,1340,1345,1350,1355,1360,
    1365,1370,1375,1380,1385,1390,1395,1400,1405,1410,
    1415,1420,1425,1430,1435,1440,1445,1450,1455,1460,
    1465,1470,1475,1480,1485,1490,1495,1500,1505,1510,
    1515,1520,1525,1530,1535,1540,1545,1550,1555,1560,
    1565,1570,1575,1580,1585,1590,1595,1600,1605,1610,
    1615,1620,1625,1630,1635,1640,1645,1650,1655,1660,
    1665,1670,1675,1680,1685,1690,1695,1700,1705,1710,
    1715,1720,1725,1730,1735,1740,1745,1750,1755,1760,
    1765,1770,1775,1780,1785,1790,1795,1800,1805,1810,
    1815,1820,1825,1830,1835,1840,1845,1850,1855,1860,
    1865,1870,1875,1880,1885,1890,1895,1900,1905,1910,
    1915,1920,1925,1930,1935,1940,1945,1950,1955,1960,
    1965,1970,1975,1980,1985,1990,1995,2000,2005,2010,
    2015,2020,2025,2030,2035,2040,2045,2050,2055,2060,
    2065,2070,2075,2080,2085,2090,2095,2100,2105,2110,
    2115,2120,2125,2130,2135,2140,2145,2150,2155,2160,
    2165,2170,2175,2180,2185,2190,2195,2200,2205,2210,
    2215,2220,2225,2230,2235,2240,2245,2250,2255,2260,
    2265,2270,2275,2280,2285,2290,2295,2300,2305,2310,
    2315,2320,2325,2330,2335,2340,2345,2350,2355,2360,
    2365,2370,2375,2380,2385,2390,2395,2400,2405,2410,
    2415,2420,2425,2430,2435,2440,2445,2450,2455,2460,
    2465,2470,2475,2480,2485,2490,2495
    ])

degree = 8  # 3次多项式示例
coefs = poly.polyfit(excitations_data, speeds_data, degree)

def get_speed_from_excitation(excitation_value):
    """
    给定激励值 excitation_value，返回估计的电机转速
    """
    speed = poly.polyval(excitation_value, coefs)
    # 做一些边界处理
    if speed < 1:
        speed = 1
    return int(speed)

###############################################################################
# 2. 电机控制类，保持你的原逻辑不变
###############################################################################
driverPUL = 12  
driverDIR = 18  
Steps_per_revolution = 790

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
            ratio = i / float(total_steps - 1)
            current_speed = start_speed + (end_speed - start_speed) * ratio
            if current_speed <= 0:
                current_speed = 1
            half_pulse_delay = 1.0 / (2.0 * current_speed)
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(half_pulse_delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(half_pulse_delay)


###############################################################################
# 3. 服务器端：等待 LabVIEW 传入激励值，计算速度，执行电机动作，并返回结果
###############################################################################
def main():
    # 创建电机对象
    motor = StepperMotor(step_pin=driverPUL, direction_pin=driverDIR)
    # 初始化：计算旋转 90 度所需要的步数（举例）
    steps_for_90_deg = int((90.0 / 360.0) * Steps_per_revolution)

    # 1) 创建 TCP Server Socket
    host_ip = "192.168.19.95"  # 监听所有网卡
    host_port = 5005     # 你可以自行选择一个空闲端口
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host_ip, host_port))
    server_socket.listen(1)
    print(f"Server is listening on {host_ip}:{host_port} ...")

    try:
        while True:
            # 2) 等待 LabVIEW 端连接
            conn, addr = server_socket.accept()
            print(f"Accepted connection from {addr}")

            # 3) 接收数据（目标激励），一般是一段字符串
            data = conn.recv(1024).decode('utf-8').strip()
            if not data:
                print("No data received, closing this connection.")
                conn.close()
                continue

            # 4) 将接收到的字符串转为浮点数(激励值)
            try:
                desired_excitation = float(data)
                print(f"Received excitation = {desired_excitation}")

                # 5) 计算得到 motor speed
                forward_speed = get_speed_from_excitation(desired_excitation)
                print(f"Converted speed = {forward_speed} steps/s")

                # 6) 驱动电机
                print("Motor rotating forward (constant speed)...")
                motor.move_steps_constant_speed(steps_for_90_deg, speed=forward_speed)

                # 这里也可以加上反转的逻辑，仅作示例
                reverse_start_speed = 300
                reverse_end_speed   = 100
                print("Motor reversing (fast -> slow)...")
                motor.move_steps_linear_decel(-steps_for_90_deg,
                                              start_speed=reverse_start_speed,
                                              end_speed=reverse_end_speed)

                # 7) 执行结束后，可以把执行结果返回给 LabVIEW
                response = f"Done, speed was {forward_speed}"
                conn.sendall(response.encode('utf-8'))

            except ValueError:
                # 如果 float(data) 出错，给 LabVIEW 返回报错
                err_msg = f"Invalid excitation data: {data}"
                print(err_msg)
                conn.sendall(err_msg.encode('utf-8'))
            except Exception as e:
                err_msg = f"Error: {e}"
                print(err_msg)
                conn.sendall(err_msg.encode('utf-8'))

            # 8) 关闭连接
            conn.close()

    except KeyboardInterrupt:
        print("Server stopped by KeyboardInterrupt.")
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        print("Cleaning up GPIO and closing server socket.")
        GPIO.cleanup()
        server_socket.close()

if __name__ == "__main__":
    main()
