#!/usr/bin/env python
from __future__ import print_function # for print without newline
import rospy # for all things ROS
from std_msgs.msg import Float64 # for motor speed messages
from sensor_msgs.msg import Joy # for game controller messages
import subprocess # for shell commands


# strings for menu
MODE_STRINGS = ["0. Teleop", "1. Start", "2. Pickup", "3. Transport"]

class Mode:
	TELEOP = 0
	START = 1
	PICKUP = 2
	TRANSPORT = 3


class Listener:
	def __init__(self):
		self.l_pub = rospy.Publisher('l_speed', Float64, queue_size=10)
		self.r_pub = rospy.Publisher('r_speed', Float64, queue_size=10)

		self.mode = Mode.TELEOP
		self.select = Mode.TELEOP

		printMenu(self.mode, self.select)

	def joy_callback(self, joy):
		if joy.axes[5] == 1: # d-pad up
			self.select -= 1
			if self.select < 0: self.select = 0

		if joy.axes[5] == -1: # d-pad down
			self.select += 1
			if self.select >= len(MODE_STRINGS): self.select = len(MODE_STRINGS) - 1

		if joy.buttons[1] == True: # X
			self.mode = self.select

		if joy.buttons[9] == True: # Start
			self.mode = Mode.TELEOP

		printMenu(self.mode, self.select)

	# teleop
	def teleop_l_speed_callback(self, teleop_l_speed):
		if self.mode == Mode.TELEOP:
			self.l_pub.publish(teleop_l_speed.data)
	def teleop_r_speed_callback(self, teleop_r_speed):
		if self.mode == Mode.TELEOP:
			self.r_pub.publish(teleop_r_speed.data)

	# start
	def start_l_speed_callback(self, start_l_speed):
		if self.mode == Mode.START:
			self.l_pub.publish(start_l_speed.data)
	def start_r_speed_callback(self, start_r_speed):
		if self.mode == Mode.START:
			self.r_pub.publish(start_r_speed.data)

	# pickup
	def pickup_l_speed_callback(self, pickup_l_speed):
		if self.mode == Mode.PICKUP:
			self.l_pub.publish(pickup_l_speed.data)
	def pickup_r_speed_callback(self, pickup_r_speed):
		if self.mode == Mode.PICKUP:
			self.r_pub.publish(pickup_r_speed.data)

	# trasport
	def transport_l_speed_callback(self, transport_l_speed):
		if self.mode == Mode.TRANSPORT:
			self.l_pub.publish(transport_l_speed.data)
	def transport_r_speed_callback(self, transport_r_speed):
		if self.mode == Mode.TRANSPORT:
			self.r_pub.publish(transport_r_speed.data)


def printMenu(mode, select):
	subprocess.call("clear")
	for i in range(0, len(MODE_STRINGS)):
		print('\033[0m', end='') # clear formatting

		if select == i:
			print(' > ', end='')
		else:
			print('   ', end='')

		if mode == i:
			print('\033[4m', end='') # underline

		print(MODE_STRINGS[i])

def interface():
	rospy.init_node('interface', anonymous=True)

	listener = Listener()

	# game controller
	rospy.Subscriber('joy', Joy, listener.joy_callback)
	
	# teleop node
	rospy.Subscriber('teleop_l_speed', Float64, listener.teleop_l_speed_callback)
	rospy.Subscriber('teleop_r_speed', Float64, listener.teleop_r_speed_callback)

	# start node
	rospy.Subscriber('start_l_speed', Float64, listener.start_l_speed_callback)
	rospy.Subscriber('start_r_speed', Float64, listener.start_r_speed_callback)

	# pickup node
	rospy.Subscriber('pickup_l_speed', Float64, listener.pickup_l_speed_callback)
	rospy.Subscriber('pickup_r_speed', Float64, listener.pickup_r_speed_callback)

	# transport node
	rospy.Subscriber('transport_l_speed', Float64, listener.transport_l_speed_callback)
	rospy.Subscriber('transport_r_speed', Float64, listener.transport_r_speed_callback)
	
	rospy.spin()

if __name__ == '__main__':
	try: interface()
	except rospy.ROSInterruptException: pass