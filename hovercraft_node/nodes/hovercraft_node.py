#!/usr/bin/python

import roslib; roslib.load_manifest('hovercraft_node')

import rospy

import sys
import math

from geometry_msgs.msg import Twist
from hovercraft_node.driver import Hovercraft
from hovercraft_node.msg import HovercraftCommand

MAX_SPEED = 1
MAX_LIFT_THRUST = 50

class HovercraftNode(object):

	def __init__(self):
		rospy.init_node('hovercraft_node')

		self.hovercraft = Hovercraft()

		self.update_rate = 30.0
		self.last_cmd_vel_time = rospy.Time.now()
		self.cmd_heading = 0
		# self.last_cmd_heading = 0
		self.cmd_left_speed = -1
		self.cmd_right_speed = -1

		self.currentLift = 0.0
		self.lastLiftThrustToggle = rospy.get_rostime()
		self.toggle = False

		self._init_pubsub()
		self._init_params()


	def _init_params(self):
		self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))

	def _init_pubsub(self):
		self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel, queue_size = 1)
		self.hovercraft_cmd_sub = rospy.Subscriber('hovercraft_cmd', HovercraftCommand, self.hovercraft_cmd, queue_size = 1)

	def start(self):
		rospy.loginfo('starting ...')

	def stop(self):
		rospy.loginfo('stopping ...')
		self.drive(0, 0)
		self.hovercraft.stop()

	def hovercraft_cmd(self, msg):
		self.last_cmd_vel_time = rospy.Time.now()

		if ((msg.leftSpeed == 0) and (msg.rightSpeed == 0)):
#                 rospy.loginfo("roll(%d, %d)" %(0, int(self.last_cmd_heading)))
			if self.cmd_left_speed != 0 or self.cmd_right_speed != 0:
				self.cmd_left_speed = 0
				self.cmd_right_speed = 0
				self.drive(0, 0, 0, 0)
		else:
			self.cmd_heading = self.normalize_angle_positive(math.atan2(msg.rightSpeed, msg.leftSpeed))*180/math.pi
			self.cmd_heading = self.cap_angle(self.cmd_heading)

			# self.cmd_left_speed = math.sqrt(math.pow(msg.leftSpeed,2)+math.pow(msg.rightSpeed,2)) / MAX_SPEED * 255
			self.cmd_left_speed = msg.leftSpeed / MAX_SPEED * 255
			self.cmd_right_speed = msg.rightSpeed / MAX_SPEED * 255

			self.cmd_heading = 0

			if msg.leftSpeed > 0:
				self.cmd_left_direction = 1
			else:
				self.cmd_left_direction = -1

			if msg.rightSpeed > 0:
				self.cmd_right_direction = 1
			else:
				self.cmd_right_direction = -1

			self.drive(int(self.cmd_left_speed), int(self.cmd_right_speed), int(self.cmd_left_direction), int(self.cmd_right_direction))
			# self.last_cmd_heading = self.cmd_heading

		currentTime = rospy.get_rostime()
		if msg.liftThrust != 0:

			if self.lastLiftThrust == 0:
				self.toggle = True
				self.lastLiftThrustToggle = rospy.get_rostime()

			if self.toggle:
			# if self.toggle and currentTime > self.lastLiftThrustToggle + rospy.Duration(0.1):

				lift = self.currentLift
				if msg.liftThrust == 1.0 and currentTime > self.lastLiftThrustToggle + rospy.Duration(0.5):
					lift = MAX_LIFT_THRUST

				elif msg.liftThrust == 0.5:
					lift = lift + 0.1 * MAX_LIFT_THRUST
					if lift > MAX_LIFT_THRUST:
						lift = MAX_LIFT_THRUST

				elif msg.liftThrust == -1.0 and currentTime > self.lastLiftThrustToggle + rospy.Duration(0.2):
					lift = 0

				elif msg.liftThrust == -0.5:
					lift = lift - 0.1 * MAX_LIFT_THRUST
					if lift < 0:
						lift = 0

				if lift != self.currentLift:
					self.toggle = False
					self.currentLift = lift
					self.setLift(int(self.currentLift))

			elif abs(msg.liftThrust) == 0.5 and currentTime > self.lastLiftThrustToggle + rospy.Duration(0.5):
				self.toggle = True
				self.lastLiftThrustToggle = rospy.get_rostime()

		else:
			self.toggle = False

		self.lastLiftThrust = msg.liftThrust

	def cmd_vel(self, msg):
		self.last_cmd_vel_time = rospy.Time.now()
		if ((msg.linear.x == 0) and (msg.linear.y == 0)):
#                 rospy.loginfo("roll(%d, %d)" %(0, int(self.last_cmd_heading)))
			if self.cmd_left_speed != 0 or self.cmd_right_speed != 0:
				self.cmd_left_speed = 0
				self.cmd_right_speed = 0
				self.drive(0, 0, 0, 0)
		else:
			self.cmd_heading = self.normalize_angle_positive(math.atan2(msg.linear.y, msg.linear.x))*180/math.pi
			self.cmd_heading = self.cap_angle(self.cmd_heading)

			# self.cmd_left_speed = math.sqrt(math.pow(msg.linear.x,2)+math.pow(msg.linear.y,2)) / MAX_SPEED * 255
			self.cmd_left_speed = msg.linear.x / MAX_SPEED * 255
			self.cmd_right_speed = msg.linear.y / MAX_SPEED * 255

			self.cmd_heading = 0

			if msg.linear.x > 0:
				self.cmd_left_direction = 1
			else:
				self.cmd_left_direction = -1

			if msg.linear.y > 0:
				self.cmd_right_direction = 1
			else:
				self.cmd_right_direction = -1

			self.drive(int(self.cmd_left_speed), int(self.cmd_right_speed), int(self.cmd_left_direction), int(self.cmd_right_direction))
			# self.last_cmd_heading = self.cmd_heading

		currentTime = rospy.get_rostime()
		if msg.linear.z != 0:

			if self.lastLiftThrust == 0:
				self.toggle = True
				self.lastLiftThrustToggle = rospy.get_rostime()

			if self.toggle:
			# if self.toggle and currentTime > self.lastLiftThrustToggle + rospy.Duration(0.1):

				lift = self.currentLift
				if msg.linear.z == 1.0 and currentTime > self.lastLiftThrustToggle + rospy.Duration(0.5):
					lift = MAX_LIFT_THRUST

				elif msg.linear.z == 0.5:
					lift = lift + 0.1 * MAX_LIFT_THRUST
					if lift > MAX_LIFT_THRUST:
						lift = MAX_LIFT_THRUST

				elif msg.linear.z == -1.0 and currentTime > self.lastLiftThrustToggle + rospy.Duration(0.2):
					lift = 0

				elif msg.linear.z == -0.5:
					lift = lift - 0.1 * MAX_LIFT_THRUST
					if lift < 0:
						lift = 0

				if lift != self.currentLift:
					self.toggle = False
					self.currentLift = lift
					self.setLift(int(self.currentLift))

			elif abs(msg.linear.z) == 0.5 and currentTime > self.lastLiftThrustToggle + rospy.Duration(0.5):
				self.toggle = True
				self.lastLiftThrustToggle = rospy.get_rostime()

		else:
			self.toggle = False

		self.lastLiftThrust = msg.linear.z


	def normalize_angle_positive(self, angle):
		return math.fmod(math.fmod(angle, 2.0*math.pi) + 2.0*math.pi, 2.0*math.pi)

	def cap_angle(self, heading):
		# return max(min(heading, 180), 0)
		# return heading
		# heading = (heading + 90) % 360
		# return max(min(heading, 180), 0)
		if heading > 180:
			return heading - 360
		else:
			return heading

	def spin(self):

		r = rospy.Rate(self.update_rate)

		while not rospy.is_shutdown():
			now = rospy.Time.now()

			if  (now - self.last_cmd_vel_time) > self.cmd_vel_timeout:
				if self.cmd_right_speed != 0 or self.cmd_left_speed != 0:
					self.cmd_right_speed = 0
					self.cmd_left_speed = 0
					# self.last_cmd_heading = 0
					self.drive(int(self.cmd_left_speed), int(self.cmd_right_speed))
				if self.currentLift != 0:
					self.currentLift = 0
					self.setLift(int(self.currentLift))
			r.sleep()

	def drive(self, left_speed, right_speed, left_direction=0, right_direction=0):
		self.hovercraft.set_left_direction(left_direction);
		self.hovercraft.set_left_speed(left_speed)
		self.hovercraft.set_right_direction(right_direction);
		self.hovercraft.set_right_speed(right_speed)
		# self.hovercraft.set_heading(heading)

	def setLift(self, thrust):
		self.hovercraft.set_lift(thrust)

def main(argv):
	h = HovercraftNode()

	while not rospy.is_shutdown():
		h.start()
		h.spin()

	h.stop()

if __name__ == '__main__':
	main(sys.argv)
