#!/usr/bin/env python
import rospy
import pigpio
from std_msgs.msg import Float64
import math
import serial

 
# Motor labeling convention:
 
#    Front
#   0     1
   
#   2     3
   
#   4     5
 
MOTOR_QUANTITY = 6
PWM_PINS = [26, 18, 20, 21, 22, 23]
DIR_PINS = [5, 6, 16, 19, 12, 13]

class Listener:
    motors = []
    def __init__(self, pi):
        for i in range (0, MOTOR_QUANTITY):
            self.motors.append(Motor(pi, PWM_PINS[i], DIR_PINS[i]))

    def lCallback(self, l_speed):
            for i in range (0, 6, 2):
                self.motors[i].setSpeed(l_speed)

    def rCallback(self, r_speed):
            for i in range (1, 6, 2):
                self.motors[i].setSpeed(r_speed)

class Motor:
    def __init__(self, pi, pwm, dir):
        self.pi = pi
        self.pwm = pwm
        self.dir = dir
        self.pi.set_mode(self.pwm, pigpio.OUTPUT)
        self.pi.set_mode(self.dir, pigpio.OUTPUT)

    def setSpeed(self, speed):

        if speed.data > 0:
            self.pi.write(self.dir, 1)

        else:
            self.pi.write(self.dir, 0)
            speed.data = -speed.data

        print(self.pi.read(16))
        rospy.loginfo("PWM Val: %f", self.dir)
        self.pi.set_PWM_dutycycle(self.pwm, speed.data * 255.0)

def motor_control():
    rospy.init_node('motor_control', anonymous = True)
    pi = pigpio.pi()
    if pi < 0:
        ROS_INFO("Error: ", pigpio_error(pi))

    listener = Listener(pi)

    rospy.Subscriber("r_speed", Float64, listener.rCallback)
    rospy.Subscriber("l_speed", Float64, listener.lCallback)

    rospy.spin()

if __name__ == '__main__':
    try: motor_control()
    except rospy.ROSInterruptException: pass