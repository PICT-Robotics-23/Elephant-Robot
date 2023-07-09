#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

def callback(botVel):
	global robotVel
	robotVel = botVel
	print(botVel)

def constrain(value, minn, maxn):
        return min(max(value, minn), maxn)

def locomotion_map(value):
	return 100 / 3.757 * value

if __name__ == '__main__':

#----------------------------------------- SETUP -------------------------------------------------#

	robotVel= Twist()
	motorAspeed = 0
	motorBspeed = 0
	motorCspeed = 0
	motorDspeed = 0
	motorApwm = 0
	motorBpwm = 0
	motorCpwm = 0
	motorDpwm = 0

	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)

	# recheck the pinout
	# motor A
	GPIO.setup(26, GPIO.OUT)
	GPIO.setup(19, GPIO.OUT)
	motorA = GPIO.PWM(19, 1000)
	motorA.start(0)

	# motor B
	GPIO.setup(6, GPIO.OUT)
	GPIO.setup(13, GPIO.OUT)
	motorB = GPIO.PWM(13, 1000)
	motorB.start(0)

	# motor C
	GPIO.setup(23, GPIO.OUT)
	GPIO.setup(18, GPIO.OUT)
	motorC = GPIO.PWM(18, 1000)
	motorC.start(0)

	# motor D
	GPIO.setup(24, GPIO.OUT)
	GPIO.setup(12, GPIO.OUT)
	motorD = GPIO.PWM(12, 1000)
	motorD.start(0)

	rospy.init_node("teleop_locomotion")
	rospy.Subscriber("cmd_vel", Twist, callback)

#----------------------------------------- LOOP --------------------------------------------------#

	while not rospy.is_shutdown():
		motorAspeed = robotVel.linear.x - robotVel.linear.y + robotVel.linear.z
		motorBspeed = robotVel.linear.x + robotVel.linear.y + robotVel.linear.z
		motorCspeed = robotVel.linear.x - robotVel.linear.y - robotVel.linear.z
		motorDspeed = robotVel.linear.x + robotVel.linear.y - robotVel.linear.z

		motorApwm = locomotion_map(motorAspeed)
		motorBpwm = locomotion_map(motorBspeed)
		motorCpwm = locomotion_map(motorCspeed)
		motorDpwm = locomotion_map(motorDspeed)

		if motorApwm < 0 :
			GPIO.output(26, GPIO.HIGH)
		else:
			GPIO.output(26, GPIO.LOW)

		if motorBpwm < 0 :
			GPIO.output(6, GPIO.HIGH)
		else:
			GPIO.output(6, GPIO.LOW)

		if motorCpwm < 0 :
			GPIO.output(23, GPIO.HIGH)
		else:
			GPIO.output(23, GPIO.LOW)

		if motorDpwm < 0 :
			GPIO.output(24, GPIO.HIGH)
		else:
			GPIO.output(24, GPIO.LOW)

		motorApwm = constrain(abs(motorApwm), 0, 100)
		motorBpwm = constrain(abs(motorBpwm), 0, 100)
		motorCpwm = constrain(abs(motorCpwm), 0, 100)
		motorDpwm = constrain(abs(motorDpwm), 0, 100)

		motorA.ChangeDutyCycle(motorApwm)
		motorB.ChangeDutyCycle(motorCpwm)
		motorC.ChangeDutyCycle(motorBpwm)
		motorD.ChangeDutyCycle(motorDpwm)


		print(motorAspeed, motorBspeed, motorCspeed, motorDpwm)

#-------------------------------------------------------------------------------------------------#

	rospy.spin()
