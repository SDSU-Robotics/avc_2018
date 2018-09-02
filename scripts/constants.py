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
WHEEL_PWM_PINS = [8, 25, 27, 23, 15, 14]
WHEEL_DIR_PINS = [7, 11, 18, 22, 17, 4]

EMAG_FIN_PIN = 8 #arbitrary
EMGA_RIN_PIN = 24 #arbitrary
