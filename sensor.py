#I2C 제어
import RPi.GPIO as GPIO
from time import sleep
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 60
LED = 0
R = 2
G = 3
B = 4
Servo = 8

def set_duty(c, duty):
	pul = duty * 16 *4096 // 100
	pca.channels[c].duty_cycle = pul


while True:
	m = input('mode')
	d = int(input('duty'))
	if m == 'l':
		set_duty(LED , d) # on LED
	elif m == 'r':
		set_duty(R, d)
	elif m == 'g':
		set_duty(G, d)
	elif m == 'b':
		set_duty(B, d)
	elif m == 's':
		set_duty(Servo, d)
	elif m == 'q':
		break

set_duty(LED, 0)
set_duty(R, 0)
set_duty(G, 0)
set_duty(B, 0)
GPIO.cleanup()

#온습도 센서 (DHT11 제어)
import time
import board
import adafruit_dht

dhtDevice = adafruit_dht.DHT22(board.D4)

while True:
    try:
        temperature_c = dhtDevice.temperature
        temperature_f = temperature_c*(9/5)+32
        humidity = dhtDevice.humidity
        print(
            "Temp:{:.1f}F/{:.1f}C Humidity:{}%".format(
                temperature_f, temperature_c, humidity
            )
        )
    except RuntimeError as error:
        print(error.args[0])
        time.sleep(2.0)
        continue



#초음파센서 (HC-SR04 제어)
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER = 22
GPIO_ECHO = 27

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def distance():
	GPIO.output(GPIO_TRIGGER,1)
	time.sleep(0.00001)
	GPIO.output(GPIO_TRIGGER,0) #트리거값을 10us동안 1로

	StartTime = time.time()
	StopTime = time.time()

	while GPIO.input(GPIO_ECHO) == 0: #에코가 마지막 0일 때 시작시간 저장
		StartTime = time.time()
	while GPIO.input(GPIO_ECHO) == 1: #에코가 마지막 1일 때 도달시간 저장
		StopTime = time.time()

	TimeElapsed = StopTime - StartTime #도달시간 - 시작시간 = 왕복시간
	distance = (TimeElapsed * 34300)/2

	return distance

while True:
	dist = distance()
	print("Measured Distance = %.1fcm" % dist)
	time.sleep(1)
GPIO.cleanup()



#라이다 센서 (YDLiDAR_X4 제어)
import os
import ydlidar
import time
#기본값세팅

ports = ydlidar.lidarPortList()
port = "/dev/ydlidar"
for key, value in ports.items():
	port = value

laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort,port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate,128000)
laser.setlidaropt(ydlidar.LidarPropLidarType,
	ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType,
	ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency,10.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate,9)
laser.setlidaropt(ydlidar.LidarPropSingleChannel,False)

ret = laser.initialize()
if ret:
	ret = laser.turnOn()
	scan = ydlidar.LaserScan()
	while ret and ydlidar.os_isOk():
		r = laser.doProcessSimple(scan)
		if r:
			print('Scan received[{}]:{}ranges is[{}]Hz'
				.format(scan.stamp, scan.points.size(),
						1.0/scan.config.scan_time)
				)
		else:
			print("Failed to get Lidar Data")
		time.sleep(0.05)
	laser.turnOff()
laser.disconnecting()



#라이다 센서 맵 출력
import os
import ydlidar
import time
import sys
from matplotlib.patches import Arc
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

RMAX = 32.0

fig = plt.figure()
fig.canvas.set_window_title('YDLidar LIDAR Monitor')
lidar_polar = plt.subplot(polar=True)
lidar_polar.autoscale_view(True, True, True)
lidar_polar.set_rmax(RMAX)
lidar_polar.grid(True)
ports = ydlidar.lidarPortList()
port = "/dev/ydlidar"
for key, value in ports.items():
    port = value

laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 10)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
laser.setlidaropt(ydlidar.LidarPropMaxRange, 32.0)
laser.setlidaropt(ydlidar.LidarPropMinRange, 0.01)
scan = ydlidar.LaserScan()


def animate(num):
    r = laser.doProcessSimple(scan);
    if r:
        angle = []
        ran = []
        intensity = []
        for point in scan.points:
            angle.append(point.angle)
            ran.append(point.range)
            intensity.append(point.intensity)
        lidar_polar.clear()
        lidar_polar.scatter(angle, ran, c=intensity, cmap='hsv', alpha=0.95)


ret = laser.initialize()
if ret:
    ret = laser.turnOn()
    if ret:
        ani = animation.FuncAnimation(fig, animate, interval=50)
        plt.show()
    laser.turnOff()
laser.disconnecting()
plt.close()


