#!/usr/bin/env python

# ros imports
import rospy
from std_msgs.msg import UInt16

# other imports
import serial
import time

ser = serial.Serial('/dev/ttyUSB0',115200,timeout = 1)

ser.write(bytes(b'B'))
ser.write(bytes(b'W'))
ser.write(bytes(2))
ser.write(bytes(0))
ser.write(bytes(0))
ser.write(bytes(0))
ser.write(bytes(1))
ser.write(bytes(6))

def sensor_input():
	pub = rospy.Publisher('LIDAR', UInt16, queue_size=10)
	rospy.init_node('sensor_input', anonymous=True)
	rate = rospy.Rate(1000) # 10hz

	distance = -1

	while not rospy.is_shutdown():
		if((b'Y' == ser.read()) and ( b'Y' == ser.read())):
			Dist_L = ser.read()
			Dist_H = ser.read()
			distance = (ord(Dist_H) * 256) + (ord(Dist_L))
			for i in range (0,5):
				ser.read()
                
		rospy.loginfo(distance)
		pub.publish(distance)
		rate.sleep()

if __name__ == '__main__':
    try:
        sensor_input()
    except rospy.ROSInterruptException:
        pass
