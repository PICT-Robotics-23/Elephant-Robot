#!/usr/bin/env python3

import RPi.GPIO as GPIO

if __name__ == '__main__':
    pick_pwm1 = -10
    pick_pwm2 = 10

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

    while(1):
        if pick_pwm1 > 0 :
            GPIO.output(0, GPIO.LOW)
        else:
            GPIO.output(0, GPIO.HIGH)
        if pick_pwm2 >0 :
            GPIO.output(5, GPIO.LOW)
        else:
            GPIO.output(5, GPIO.HIGH)

        motorA.ChangeDutyCycle(10)
        motorB.ChangeDutyCycle(10)

        




