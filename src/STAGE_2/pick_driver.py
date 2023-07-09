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
    pick_pwm1 = 0
    pick_pwm2 = 0

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # pick motor 1
    GPIO.setup(0, GPIO.OUT)
    GPIO.setup(19, GPIO.OUT)
    motorA = GPIO.PWM(19, 1000)
    motorA.start(0)

    # pick motor 2
    GPIO.setup(5, GPIO.OUT)
    GPIO.setup(13, GPIO.OUT)
    motorB = GPIO.PWM(13, 1000)
    motorB.start(0)

    rospy.init_node('pick_driver', anonymous=True)
    sub = rospy.Subscriber('cmd_motors', motorArray, callback) 

    rospy.sleep(2)
    while not rospy.is_shutdown():
        for item in motorVal.motors:
            if item.name == "pick":
                pick_pwm1 = item.power
                pick_pwm2 = -(pick_pwm1)
                break

        if pick_pwm1 < 0 :
            GPIO.output(0, GPIO.HIGH)
            pick_pwm2 = float(pick_pwm2)
            pick_pwm2 *= 1.4    
        else:
            GPIO.output(0, GPIO.LOW)
            pick_pwm2 = float(pick_pwm2)
            pick_pwm2 /= 1.4

        if pick_pwm2 < 0 :
            GPIO.output(5, GPIO.HIGH)
        else:
            GPIO.output(5, GPIO.LOW)
            

        print(pick_pwm1, pick_pwm2)

        pick_pwm1 = value_mapping(abs(pick_pwm1), 0, 255, 0, 100)
        pick_pwm2 = value_mapping(abs(pick_pwm2), 0, 255, 0, 100)

        motorA.ChangeDutyCycle(pick_pwm1)
        motorB.ChangeDutyCycle(pick_pwm2)

    rospy.spin()
        




