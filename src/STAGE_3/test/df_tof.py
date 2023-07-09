from raspberry.DFRobot_TMF8x01 import DFRobot_TMF8701 as tof

import sys
import os
import time

tof = tof(enPin=-1, intPin=-1, bus_id=1)

if __name__ == "__main__":
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

    while True:
      if(tof.is_data_ready() == True):
        print("Distance = %d mm"%tof.get_distance_mm())

