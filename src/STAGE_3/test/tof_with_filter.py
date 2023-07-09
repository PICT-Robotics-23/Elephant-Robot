from raspberry.DFRobot_TMF8x01 import DFRobot_TMF8701 as tof

import sys
import os
import time

tof = tof(enPin=-1, intPin=-1, bus_id=1)

from scipy.signal import butter, lfilter

def butter_lowpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def apply_filter(data, cutoff_freq, sampling_freq, filter_order=5):
    b, a = butter_lowpass(cutoff_freq, sampling_freq, order=filter_order)
    filtered_data = lfilter(b, a, data)
    return filtered_data


if __name__ == "__main__":

    cutoff_freq = 15  
    sampling_freq = 1000  
    filter_order = 4  

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
        filtered_data = apply_filter(tof.get_distance_mm, cutoff_freq, sampling_freq, filter_order)
        print(filtered_data)

