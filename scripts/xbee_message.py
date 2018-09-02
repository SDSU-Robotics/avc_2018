#! /usr/bin/env python

from xbee import XBee, ZigBee
import serial
import rospy
from std_msgs.msg import String

ser = serial.Serial('/dev/ttyUSB0', 9600)



# Use an XBee 802.15.4 device
xbee = XBee(ser)
# To use with an XBee ZigBee device, replace with:
# xbee = ZigBee(ser)


def xbee_message():

    xbee_msg = rospy.Publisher('xbee_message', String, queue_size=10)
    rospy.init_node('xbee', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

            msg = xbee.wait_read_frame()
            rospy.loginfo(msg['rf_data'])
            xbee_msg.publish(msg['rf_data'])
            rate.sleep()



if __name__ =='__main__':
    try:
        xbee_message()
    except rospy.ROSInterruptException:
        pass

ser.close()









