#!/usr/bin/env python

# ros imports
import rospy
from std_msgs.msg import UInt16

# other imports
import serial
import time

def sensor_input():
	# create publishers
	pub = [rospy.Publisher('LIDAR0', UInt16, queue_size=10),
		rospy.Publisher('LIDAR1', UInt16, queue_size=10),
		rospy.Publisher('LIDAR2', UInt16, queue_size=10)]

	# initialize node
	rospy.init_node('sensor_input', anonymous=True)
	rate = rospy.Rate(1000) # 10hz

	# try to open serial ports
	ser = [0, 0, 0]

	try: ser[0] = serial.Serial('/dev/ttyUSB0',115200,timeout = .1)
	except: rospy.logerr("Failed to open /dev/ttyUSB0")

	try: ser[1] = serial.Serial('/dev/ttyUSB1',115200,timeout = .1)
	except: rospy.logerr("Failed to open /dev/ttyUSB1")

	try: ser[2] = serial.Serial('/dev/ttyUSB2',115200,timeout = .1)
	except: rospy.logerr("Failed to open /dev/ttyUSB2")

	# set mode of LIDARs
	for x in ser:
		if x:
			x.write(bytes(b'B'))
			x.write(bytes(b'W'))
			x.write(bytes(2))
			x.write(bytes(0))
			x.write(bytes(0))
			x.write(bytes(0))
			x.write(bytes(1))
			x.write(bytes(6))
	
	# set default distance
	distance = [65535, 65535, 65535]

	while not rospy.is_shutdown():
		for i in range(0, 3):
			# if successfully opened
			if ser[i] != 0:
				# check for start bytes
				if((b'Y' == ser[i].read()) and ( b'Y' == ser[i].read())):	
					Dist_L = ser[i].read()
					Dist_H = ser[i].read()
					distance[i] = (ord(Dist_H) * 256) + (ord(Dist_L))
					# discard remaining 5 bytes
					for j in range (0,5):
						ser[i].read()
                
			rospy.loginfo("LIDAR%d: %d", i, distance[i])
			pub[i].publish(distance[i])
		rate.sleep()

if __name__ == '__main__':
    try:
        sensor_input()
    except rospy.ROSInterruptException:
        pass
