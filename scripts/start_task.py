#! /usr/bin/env python

import serial
import rospy # Import for ROS Python
from std_msgs.msg import String # For XBee messages
from std_msgs.msg import Float64 # For motor_control messages
import time


class Listener:
    def __init__(self):
        self.l_pub = rospy.Publisher('start_l_speed', Float64, queue_size=10)
        self.r_pub = rospy.Publisher('start_r_speed', Float64, queue_size=10)

    def callback(self, message):


        t_end = time.time() + 3 # Get current time and add three seconds

        if message.data == 'GO':

            # Begin moving for 3 seconds
            self.l_pub.publish(0.7)
            self.r_pub.publish(0.7)

            # 3 Second while loop
            while time.time() < t_end:
                None


            # After 3 seconds stop moving
            self.l_pub.publish(0.0)
            self.r_pub.publish(0.0)

def start():

    rospy.init_node('start_task', anonymous = True)

    listener = Listener()

    rospy.Subscriber('xbee_message', String, listener.callback)

    rospy.spin()

if __name__ == '__main__':
    try: start()
    except rospy.ROSInterruptException: pass



