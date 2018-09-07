#! /usr/bin/env python

from xbee import XBee, ZigBee
import serial
import rospy
from std_msgs.msg import String
import constants



def xbee_message():
    xbee_msg = rospy.Publisher('xbee_message', String, queue_size=10)
    rospy.init_node('xbee', anonymous=False)
    rate = rospy.Rate(1000)

    # try to open serial ports
    try: ser = serial.Serial(constants.XBEE_PORT, 115200, timeout = .1)
    except: rospy.logerr("Failed to open %s", constants.XBEE_PORT)    

    i = 0

    while not rospy.is_shutdown():
        if ser != 0:
            #read xbee data

            # Use an XBee 802.15.4 device
            xbee = XBee(ser)

            msg = xbee.wait_read_frame()   
 
            i = i + 1
            print(i)

            #rospy.loginfo(msg['rf_data'])
            xbee_msg.publish(msg['rf_data'])
        
    rate.sleep()

       

if __name__ =='__main__':
    try:
        xbee_message()
    except rospy.ROSInterruptException:
        ser.close()











