#!/usr/bin/env python3

import sys
import os
import time
import rospy
import math
from mpu6050 import mpu6050
from raspberry.DFRobot_TMF8x01 import DFRobot_TMF8701 as tof
from time import sleep

sensor = mpu6050(0x68)
tof = tof(enPin=-1, intPin=-1, bus_id=1)

roll=0
pitch=0
yaw = 0

def callback(msg):
    global motorVal
    motorVal = msg

def constrain(value, minn, maxn):
    return min(max(value, minn), maxn)

def calibrateTOF():
    print("Initialization ranging sensor TMF8x01......", end = " ")
    while(tof.begin() != 0):
        print("Initialization failed")
        time.sleep(1)
    print("Initialization done.")

    calibrationList = []
    
    while (len(calibrationList) != 14):
      print("Get and print calibration...")
      calibrationList = tof.get_calibration_data()
      time.sleep(1)
    
    print("Get and print calibration...sucess")
    print(calibrationList)
    tof.set_calibration_data(calibrationList)
    
    if tof.start_measurement(calib_m = tof.eMODE_CALIB, mode = tof.ePROXIMITY) != True:
      print("Enable measurement faild.\nPlease check the calibration data and recalibrate!!!")

def readMPU():
    global roll, pitch, yaw

    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()

    ax = accel_data['x']
    ay = accel_data['y']
    az = accel_data['z']
    gx = gyro_data['x']
    gy = gyro_data['y']
    gz = gyro_data['z']

    roll = math.atan2(ay, az) * (180 / math.pi)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * (180 / math.pi)
    yaw = math.atan2(gy, gx) * (180 / math.pi)

if __name__ == '__main__':

    tof_distance = 0
    calibrateTOF()
   
    while not rospy.is_shutdown():
        
        if(tof.is_data_ready() == True):
            tof_distance = tof.get_distance_mm()

        readMPU()

        print("tof: ", tof_distance)
        print("roll: ", roll, " pitch: ", pitch, " yaw: ",yaw)

    rospy.spin()
        




