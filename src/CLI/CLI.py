#!/usr/bin/env python3

import argparse
import sys
import fontstyle
import rospy
from os import system
from time import sleep
from sensor_msgs.msg import Range
from elephant_robot.msg import motorArray
from std_msgs.msg import Int32

shoot_val = 0

def displaydata(dist: int, sh: int, ang: int):
    try:
        global tof_distance, shoot, shoot_val, angle
        tof_heading = fontstyle.apply("TOF", "bold/cyan")
        shoot_heading = fontstyle.apply("SHOOT", "bold/cyan")
        angle_heading = fontstyle.apply("ANGLE", "bold/cyan")

        tof_value = str(((tof_distance.range) % 14) + 6)[:4].center(5)  
        angle_value = str(angle.data)[:4].center(5)
        for item in shoot.motors:
            if item.name == "shoot":
                shoot_val = item.power
                break
        shoot_value = str(shoot_val).center(7)  

        heading_width = 15  

        print("+-------+---------+-------+")
        print(f"|  {tof_heading.center(heading_width)}  |  {shoot_heading.center(heading_width)}  | {angle_heading.center(heading_width)} |")
        print("+-------+---------+-------+")
        print(f"| {tof_value} | {shoot_value} | {angle_value} |")
        print("+-------+---------+-------+")
        sleep(1)
        _ = system('clear')
    except KeyboardInterrupt:
        return

def callback0(msg):
    global tof_distance
    tof_distance = msg

def callback1(msg):
    global shoot
    shoot = msg

def callback2(msg):
    global angle
    angle = msg

if __name__ == "__main__":

    tof_distance = Range()
    shoot = motorArray()
    angle = Int32()

    rospy.init_node('cli', anonymous=True)
    sub0 = rospy.Subscriber('tof_dist', Range, callback0) 
    sub1 = rospy.Subscriber('cmd_motors', motorArray, callback1)
    sub2 = rospy.Subscriber('mpu_angle', Int32, callback2)


    parser = argparse.ArgumentParser(description="Display python version using CLI.")
    parser.add_argument("-d", "--Distance", type=int, help= "1 to display 0 to leave", default=1)
    parser.add_argument("-s", "--Shoot", type=int, help= "1 to display 0 to leave", default=1)
    parser.add_argument("-a", "--Angle", type=int, help= "1 to display 0 to leave", default=1)
    args = parser.parse_args()

    while not rospy.is_shutdown():
        displaydata(args.Distance, args.Shoot, args.Angle)

    rospy.spin()
