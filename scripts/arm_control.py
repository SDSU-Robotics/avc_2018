#!/usr/bin/env python
import rospy
import pigpio
from std_msgs.msg import Float64
import math
import serial
import constants

class Servo:
    def __init__(self, pi, pin):
        self.pi = pi
        self.pin = pin
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(self.pin, 313)

    def setPosition(self, pos):
        if pos < 0:
            pos = 0

        pos = pos * (constants.MAX_SERVO_PWM - constants.MIN_SERVO_PWM) + constants.MIN_SERVO_PWM

        if pos > constants.MAX_SERVO_PWM:
            pos = constants.MAX_SERVO_PWM

        rospy.loginfo("Pin %d PWM Val: %f", self.pin, pos)
        self.pi.set_PWM_dutycycle(self.pin, pos)

class Listener:
    def __init__(self, pi):
        self.base = Servo(pi, constants.BASE_PIN)
        self.joint0 = Servo(pi, constants.JOINT0_PIN)
        self.joint1 = Servo(pi, constants.JOINT1_PIN)

        self.base.setPosition(constants.BASE_START_POS)
        self.joint0.setPosition(constants.JOINT0_START_POS)
        self.joint1.setPosition(constants.JOINT1_START_POS)

    def baseCallback(self, base_pos):
        self.base.setPosition(base_pos.data)

    def joint0Callback(self, joint0_pos):
        self.joint0.setPosition(joint0_pos.data)

    def joint1Callback(self, joint1_pos):
        self.joint1.setPosition(joint1_pos.data)

def motor_control():
    rospy.init_node('motor_control', anonymous = True)

    pi = pigpio.pi()
    if pi < 0:
        rospy.logerr("Error: ", pigpio_error(pi))

    listener = Listener(pi)

    rospy.Subscriber("base_pos", Float64, listener.baseCallback)
    rospy.Subscriber("joint1_pos", Float64, listener.joint0Callback)
    rospy.Subscriber("joint0_pos", Float64, listener.joint1Callback)

    rospy.spin()

if __name__ == '__main__':
    try: motor_control()
    except rospy.ROSInterruptException: pass
