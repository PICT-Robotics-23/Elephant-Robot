#!/usr/bin/python3

from mpu6050 import mpu6050
from time import sleep
import math

sensor = mpu6050(0x68)

while True:
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    temp = sensor.get_temp()

    ax = accel_data['x']
    ay = accel_data['y']
    az = accel_data['z']
    gx = gyro_data['x']
    gy = gyro_data['y']
    gz = gyro_data['z']

    roll = math.atan2(ay, az) * (180 / math.pi)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * (180 / math.pi)
    yaw = math.atan2(gy, gx) * (180 / math.pi)

    print("Values: ")
    print("Roll: " + str(roll))
    print("Pitch: " + str(pitch))
    print("Yaw: " + str(yaw))
    sleep(0.5)
