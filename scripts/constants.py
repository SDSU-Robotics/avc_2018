#!/usr/bin/env python

LIDAR0_PORT = "/dev/ttyUSB0"
LIDAR1_PORT = "/dev/ttyUSB1"
LIDAR2_PORT = "/dev/ttyUSB2"

XBEE_PORT = "/dev/ttyUSB3"

# Motor labeling convention:

#    Front
#   0     1

#   2     3

#   4     5

MOTOR_QUANTITY = 6
WHEEL_PWM_PINS = [26, 18, 20, 21, 22, 23]
WHEEL_DIR_PINS = [5, 7, 16, 19, 12, 13]

EMAG_FIN_PIN = 8 #arbitrary
EMGA_RIN_PIN = 24 #arbitrary

# Computer vision
IPP = 0.017188
IMAGE_W = 1280
IMAGE_H = 960
A = 1
B = -1
