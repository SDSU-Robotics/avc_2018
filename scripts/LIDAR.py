#!/usr/bin/env python

# ros imports
import rospy
from std_msgs.msg import UInt16

# other imports
import constants
import serial
import time

def sensor_input():
	# create publishers
	pub = rospy.Publisher('lidar', UInt16, queue_size=10)

	# initialize node
	rospy.init_node('sensor_input', anonymous=True)
	rate = rospy.Rate(1000) # 1000 hz

	# try to open serial ports
	try: ser = serial.Serial(constants.LIDAR_PORT, 115200, timeout = .1)
	except: rospy.logerr("Failed to open %s", constants.LIDAR_PORT)

	# set mode of LIDARs
	if ser:
		ser.write(bytes(b'B'))
		ser.write(bytes(b'W'))
		ser.write(bytes(2))
		ser.write(bytes(0))
		ser.write(bytes(0))
		ser.write(bytes(0))
		ser.write(bytes(1))
		ser.write(bytes(6))
	
	# set default distance
	distance = 65535
	strength = 0

	while not rospy.is_shutdown():
		
			# if successfully opened
			if ser != 0:
				# check for start bytes
				if((b'Y' == ser.read()) and ( b'Y' == ser.read())):	
					Dist_L = ser.read()
					Dist_H = ser.read()
					StrengthL = ser.read()
					StrengthH = ser.read()
					distance = (ord(Dist_H) * 256) + (ord(Dist_L))
					strength = (ord(StrengthH) * 256) + (ord(StrengthL))

					# discard remaining 3 bytes
					for j in range (0,3):
						ser.read()
                
			#rospy.loginfo("LIDAR%d: %d", i, distance[i])
			if strength > 100:
				pub.publish(distance)
			else:
				pub.publish(65535)
		
	rate.sleep()

if __name__ == '__main__':
    try:
        sensor_input()
    except rospy.ROSInterruptException:
        pass
