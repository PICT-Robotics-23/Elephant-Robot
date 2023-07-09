#!/usr/bin/env python3

import rospy
import serial
import re
from elephant_robot.msg import motor, motorArray

port = '/dev/ttyUSB0'  
baud_rate = 115200     

# picking 
pick_up = 0
pick_down = 0
up_factor = 30
down_factor = 30
pick_mode = 0
pick_speed = 2
pick_pwm = 0.0

# loading
load_in = 0
load_out = 0
in_factor = 85
out_factor = 20

# shooting
shoot_speed = 0

def getValues(str):
    global pick_up, pick_down, load_in, load_out, shoot_speed, pick_mode
    values = re.findall(r'-?\d+', str[1:])
    pick_up = int(values[0])
    pick_down = int(values[1])
    load_in = int(values[2])
    load_out = int(values[3])
    shoot_speed = int(values[4])
    pick_mode = int(values[5])


def serial_node():
    rospy.init_node('esp_ros_bridge', anonymous=True)
    pub = rospy.Publisher('cmd_motors', motorArray, queue_size=10)
    ser = serial.Serial(port, baud_rate)

    motor_load = motor()
    motor_pick = motor()
    motor_shoot = motor()

    motor_array = motorArray()

    rospy.sleep(2)
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').rstrip()
            if data:
                if data[0] == "6":
                    getValues(data)
                    pick_pwm = (((up_factor * pick_up) or (down_factor * pick_down)) * (pick_up - pick_down)) * ((pick_speed * pick_mode) or (1 *  (not pick_mode)))
                    load_pwm = (((in_factor * load_in) or (out_factor * load_out)) * (load_in - load_out))

                    motor_pick.name = "pick"
                    motor_pick.power = int(pick_pwm)

                    motor_load.name = "load"
                    motor_load.power = load_pwm

                    motor_shoot.name = "shoot"
                    motor_shoot.power = shoot_speed

                    motor_array.motors.append(motor_pick)
                    motor_array.motors.append(motor_load)   
                    motor_array.motors.append(motor_shoot)
                                    
                    pub.publish(motor_array) 
                    
                    motor_array.motors.clear()
                    

if __name__ == '__main__':
    try:
        serial_node()
    except rospy.ROSInterruptException:
        pass

