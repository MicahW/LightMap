from Controller import *
import sys

# Test script for motor messaging
# Run with arguments python TestMotor.py <right motor speed> <left motor speed>
# Backwards is not supported

right_speed = int(sys.argv[1])
left_speed = int(sys.argv[2])

print("{}, {}".format(right_speed, left_speed))

controller = Controller()
controller.motor(True, right_speed, True, left_speed)
