#!/usr/bin/env python3

import sys
import os
import time
import rospy
import math
from mpu6050 import mpu6050
from std_msgs.msg import Int32

sensor = mpu6050(0x68)

roll=0
pitch=0
yaw = 0

def readMPU():
    global roll, pitch, yaw
    try:
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

        return -(int(pitch))
    except:
        return(0)

if __name__ == '__main__':
    rospy.init_node('mpu_interface', anonymous=True)
    pub = rospy.Publisher('mpu_angle', Int32, queue_size=10)

    mpu_angle = 0


   
    while not rospy.is_shutdown():

        mpu_angle = readMPU()

        print(mpu_angle)
        pub.publish(mpu_angle)

    rospy.spin()