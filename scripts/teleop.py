#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8

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
	rate = rospy.Rate(50) # 50 hz

	listener = Listener()
	buttons = [0] * 12
	axes = [0] * 6

	rospy.Subscriber('joy', Joy, listener.joyCallback)
	
	l_pub = rospy.Publisher('teleop_l_speed', Float64, queue_size=10)
	r_pub = rospy.Publisher('teleop_r_speed', Float64, queue_size=10)
	emag_pub = rospy.Publisher('teleop_emag', UInt8, queue_size=10) 
	rho_pub = rospy.Publisher('teleop_rho', Float64, queue_size=10)
	phi_pub = rospy.Publisher('teleop_phi', Float64, queue_size=10)
	z_pub = rospy.Publisher('teleop_z', Float64, queue_size=10)

	while not rospy.is_shutdown():
		listener.getJoyValues(buttons, axes)
		
		if buttons[5] == True:
			rho_pub.publish(axes[3])
			phi_pub.publish(axes[2])
			z_pub.publish(axes[1])
		else:
			l_pub.publish(axes[1])
			r_pub.publish(axes[3])

		if buttons[3] == True: # whatever that is
			emag_pub.publish(2) # push
		if buttons[2] == True:
			emag_pub.publish(0) # off
		if buttons[1] == True:
			emag_pub.publish(1) # pull




		rate.sleep()

if __name__ == '__main__':
	try: teleop()
	except rospy.ROSInterruptException: pass
