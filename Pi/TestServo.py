from Controller import *
import time

controller = Controller()
while True:
    controller.servo(80);
    time.sleep(3)
    controller.servo(-80);
    time.sleep(3)
    