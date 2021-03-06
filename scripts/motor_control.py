#!/usr/bin/env python
import rospy
import pigpio
from std_msgs.msg import Float64
import math
import serial
import constants


# Motor labeling convention:

#    Front
#   0     1

#   2     3

#   4     5

class Motor:
    def __init__(self, pi, pwm, dirPin):
        self.pi = pi
        self.pwm = pwm
        self.dirPin = dirPin
        self.pi.set_mode(self.pwm, pigpio.OUTPUT)
        self.pi.set_mode(self.dirPin, pigpio.OUTPUT)

    def setSpeed(self, speed):
        speed = speed * 255
#	rospy.loginfo("Speed: %f", speed)
        if speed > 0:
            self.pi.write(self.dirPin, 1)

        else:
            self.pi.write(self.dirPin, 0)
            speed = -speed

        if speed > 255:
            speed = 255

        rospy.loginfo("PWM Val: %f  Dir: %d  PWM %d", speed, self.dirPin, self.pwm)
        self.pi.set_PWM_dutycycle(self.pwm, speed)

class Listener:
    def __init__(self, pi):
        self.motors = []
        for i in range(0, constants.MOTOR_QUANTITY):
            self.motors.append(Motor(pi, constants.WHEEL_PWM_PINS[i], constants.WHEEL_DIR_PINS[i]))
            self.motors[i].setSpeed(0)

    def lCallback(self, l_speed):
        for i in range(0, constants.MOTOR_QUANTITY, 2):
            rospy.loginfo("L Speed %f", l_speed.data)
	    self.motors[i].setSpeed(l_speed.data)

    def rCallback(self, r_speed):
        for i in range(1, constants.MOTOR_QUANTITY, 2):
            self.motors[i].setSpeed(r_speed.data)

def motor_control():
    rospy.init_node('motor_control', anonymous = True)

    pi = pigpio.pi()
    if pi < 0:
        rospy.logerr("Error: ", pigpio_error(pi))

    listener = Listener(pi)

    rospy.Subscriber("r_speed", Float64, listener.rCallback)
    rospy.Subscriber("l_speed", Float64, listener.lCallback)
   
	
    rospy.spin()

if __name__ == '__main__':
    try: motor_control()
    except rospy.ROSInterruptException: pass
