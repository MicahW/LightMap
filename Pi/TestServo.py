from Controller import *
import time

controller = Controller()
while True:
    controller.servo(70);
    time.sleep(3)
    controller.servo(-70);
    time.sleep(3)
    