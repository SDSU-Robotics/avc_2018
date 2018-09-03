#!/usr/bin/env python

LIDAR_PORT = "/dev/ttyUSB0"
#LIDAR1_PORT = "/dev/ttyUSB1"
#LIDAR2_PORT = "/dev/ttyUSB2"

#XBEE_PORT = '/dev/ttyUSB2'

# Motor labeling convention:

#    Front
#   0     1

#   2     3

#   4     5

MOTOR_QUANTITY = 6
WHEEL_PWM_PINS = [26, 18, 20, 21, 22, 23]
WHEEL_DIR_PINS = [5, 7, 16, 19, 12, 13]

MAX_ESC_PWM = 255

# Servos
BASE_PIN = 26
JOINT0_PIN = 20
JOINT1_PIN = 21

BASE_START_POS = 0.8
JOINT0_START_POS = 0.5
JOINT1_START_POS = 0.5

MAX_SERVO_PWM = 200
MIN_SERVO_PWM = 40

# Electromagnet
EMAG_FIN_PIN = 8 #arbitrary
EMGA_RIN_PIN = 24 #arbitrary
