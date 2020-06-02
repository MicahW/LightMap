from Controller import *
import sys
import time

# Test script for motor messaging

controller = Controller()
controller.ping()
controller.move_straight()
time.sleep(3)
controller.stop()