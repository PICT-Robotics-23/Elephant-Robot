#!/usr/bin/env python3

import sys
import os
import time
import rospy
from sensor_msgs.msg import Range
from raspberry.DFRobot_TMF8x01 import DFRobot_TMF8701 as tof

tof = tof(enPin=-1, intPin=-1, bus_id=1)

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
    
    if tof.start_measurement(calib_m = tof.eMODE_CALIB, mode = tof.eCOMBINE) != True:
      print("Enable measurement faild.\nPlease check the calibration data and recalibrate!!!")

if __name__ == '__main__':
    rospy.init_node('tof_interface', anonymous=True)
    pub = rospy.Publisher('tof_dist', Range, queue_size=10)

    calibrateTOF()
    tof_distance = Range()

    while not rospy.is_shutdown():
        
        if(tof.is_data_ready() == True):
            tof_distance.range = tof.get_distance_mm()

        print("tof: ", tof_distance.range)
        pub.publish(tof_distance)

    rospy.spin()




