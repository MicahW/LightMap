from Controller import *
import sys
import time

# Test script for motor messaging

controller = Controller()
try:
    controller.ping()
    controller.move_straight()
    while True:
        time.sleep(0.2)
        v = controller.ultrasonic()
        print(v)
        if v < 3000:
            controller.stop()
            break
except Exception as e:
    controller.stop()
    print(e)
