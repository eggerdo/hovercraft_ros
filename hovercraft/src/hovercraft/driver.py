#!/usr/bin/python

import rospy

from Tkinter import *
import RPi.GPIO as GPIO
import time

LIFT_PWM_PIN               = 4
LIFT_DIRECTION_PIN         = 27
LEFT_MOTOR_PWM_PIN         = 18
LEFT_MOTOR_DIRECTION_PIN   = 23
RIGHT_MOTOR_PWM_PIN        = 7
RIGHT_MOTOR_DIRECTION_PIN  = 25

class Hovercraft(object):

	def __init__(self):
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(LEFT_MOTOR_PWM_PIN, GPIO.OUT)
		GPIO.setup(LEFT_MOTOR_DIRECTION_PIN, GPIO.OUT)
		GPIO.setup(RIGHT_MOTOR_PWM_PIN, GPIO.OUT)
		GPIO.setup(RIGHT_MOTOR_DIRECTION_PIN, GPIO.OUT)
		GPIO.setup(LIFT_PWM_PIN, GPIO.OUT)
		GPIO.setup(RIGHT_MOTOR_DIRECTION_PIN, GPIO.OUT)

		GPIO.output(RIGHT_MOTOR_DIRECTION_PIN, GPIO.HIGH)

		self.left_speed_pwm = GPIO.PWM(LEFT_MOTOR_PWM_PIN, 100)
		self.left_speed_pwm.start(0)
		self.right_speed_pwm = GPIO.PWM(RIGHT_MOTOR_PWM_PIN, 100)
		self.right_speed_pwm.start(0)
		self.lift_pwm = GPIO.PWM(LIFT_PWM_PIN, 100)
		self.lift_pwm.start(0)

		self.old_left_direction = 0
		self.old_right_direction = 0
		# self.left_speed_pwm.start(5)

	def stop(self):
		GPIO.cleanup()

	def set_left_speed(self, speed):
		speed = abs(speed)
		rospy.loginfo('set left speed: %d' %speed)
		dutyCycle = speed * 100.0 / 255
		# rospy.loginfo('duty cycle: %d' %dutyCycle)
		self.left_speed_pwm.ChangeDutyCycle(dutyCycle)
		pass

	def set_right_speed(self, speed):
		speed = abs(speed)
		rospy.loginfo('set right speed: %d' %speed)
		dutyCycle = speed * 100.0 / 255
		# rospy.loginfo('duty cycle: %d' %dutyCycle)
		self.right_speed_pwm.ChangeDutyCycle(dutyCycle)
		pass

	def set_left_direction(self, direction):
		if direction >= 0 and self.old_left_direction < 0:
			# rospy.loginfo('switch direction: %d, old_left_direction: %d' %(direction, self.old_left_direction))
			GPIO.output(LEFT_MOTOR_DIRECTION_PIN, GPIO.HIGH)
			self.old_left_direction = direction
		elif direction < 0 and self.old_left_direction >= 0:
			# rospy.loginfo('switch direction: %d, old_left_direction: %d' %(direction, self.old_left_direction))
			GPIO.output(LEFT_MOTOR_DIRECTION_PIN, GPIO.LOW)
			self.old_left_direction = direction

	def set_right_direction(self, direction):
		if direction >= 0 and self.old_right_direction < 0:
			# rospy.loginfo('switch direction: %d, old_left_direction: %d' %(direction, self.old_left_direction))
			GPIO.output(RIGHT_MOTOR_DIRECTION_PIN, GPIO.HIGH)
			self.old_right_direction = direction
		elif direction < 0 and self.old_right_direction >= 0:
			# rospy.loginfo('switch direction: %d, old_left_direction: %d' %(direction, self.old_left_direction))
			GPIO.output(RIGHT_MOTOR_DIRECTION_PIN, GPIO.LOW)
			self.old_right_direction = direction

	def set_lift(self, thrust):
		thrust = abs(thrust)
		rospy.loginfo('set lift thrust: %d' %thrust)
		# dutyCycle = thrust * 100.0
		dutyCycle = thrust
		rospy.loginfo('duty cycle: %d' %dutyCycle)
		self.lift_pwm.ChangeDutyCycle(dutyCycle)

	def set_heading(self, heading):
		rospy.loginfo('set heading: %d' %heading)
		pass

