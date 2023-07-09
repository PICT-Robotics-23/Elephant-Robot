#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from elephant_robot.msg import motorArray

def callback(msg):
    global motorVal
    motorVal = msg

def constrain(value, minn, maxn):
    return min(max(value, minn), maxn)

def value_mapping(value, in_min, in_max, out_min, out_max):
    return (value-in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == '__main__':
    motorVal = motorArray()
    load_pwm1 = 0
    load_pwm2 = 0

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # pick motor 1
    GPIO.setup(23, GPIO.OUT)
    GPIO.setup(18, GPIO.OUT)
    motorA = GPIO.PWM(18, 1000)
    motorA.start(0)

    # pick motor 2
    GPIO.setup(24, GPIO.OUT)
    GPIO.setup(12, GPIO.OUT)
    motorB = GPIO.PWM(12, 1000)
    motorB.start(0)

    # limit switch
    GPIO.setup(21, GPIO.IN)
    GPIO.setup(8, GPIO.IN)
    GPIO.setup(25, GPIO.IN)
    GPIO.setup(7, GPIO.IN)

    rospy.init_node('load_driver', anonymous=True)
    sub = rospy.Subscriber('cmd_motors', motorArray, callback) 

    rospy.sleep(2)
    while not rospy.is_shutdown():

        for item in motorVal.motors:
            if item.name == "load":
                load_pwm1 = item.power
                load_pwm2 = -(load_pwm1)
                break
        
        limit_in_r = GPIO.input(21)
        limit_in_l = GPIO.input(25)
        limit_out_r = GPIO.input(8)
        limit_out_l = GPIO.input(7)

        print(limit_in_r, limit_in_l, limit_out_r, limit_out_l)

        if load_pwm1 > 0 and (not limit_out_r):
            load_pwm1 = 0
        elif load_pwm1 < 0 and (not limit_in_r):
            load_pwm1 = 0
        if load_pwm2 > 0 and (not limit_in_l):
            load_pwm2 = 0
        elif load_pwm2 < 0 and (not limit_out_l):
            load_pwm2 = 0


        if load_pwm1 > 0 :
            GPIO.output(23, GPIO.LOW)
        else:
            GPIO.output(23, GPIO.HIGH)

        if load_pwm2 > 0 :
            GPIO.output(24, GPIO.HIGH)
        else:
            GPIO.output(24, GPIO.LOW)

        load_pwm1 = value_mapping(abs(load_pwm1 * 0.9), 0, 255, 0, 100)
        load_pwm2 = value_mapping(abs(load_pwm2), 0, 255, 0, 100)

        motorA.ChangeDutyCycle(load_pwm1)
        motorB.ChangeDutyCycle(load_pwm2)

    rospy.spin()
        




