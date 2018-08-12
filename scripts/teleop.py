#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy


class Listener:
	buttons = [0] * 12
	axes = [0] * 6

	def joyCallback(self, joy):
		for i in range(0, 12):
			self.buttons[i] = joy.buttons[i]
		for i in range (0, 6):
			self.axes[i] = joy.axes[i]

	def getJoyValues(self, buttons, axes):
		for i in range(0, 12):
			buttons[i] = self.buttons[i]
		for i in range (0, 6):
			axes[i] = self.axes[i]

def teleop():
	rospy.init_node('teleop', anonymous=True)
	
	l_pub = rospy.Publisher('teleop_l_speed', Float64, queue_size=10)
	r_pub = rospy.Publisher('teleop_r_speed', Float64, queue_size=10)
	
	listener = Listener()

	rospy.Subscriber("joy", Joy, listener.joyCallback)

	rospy.spin()


if __name__ == '__main__':
	try: teleop()
	except rospy.ROSInterruptException: pass
