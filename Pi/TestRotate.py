import time
import logging

from Controller import Controller

logger = logging.getLogger('Controller')

logger.addHandler(logging.StreamHandler())
logger.setLevel(logging.DEBUG)

controller = Controller()
while True:
    controller.rotate(clockwise=True)
    time.sleep(3)

    controller.rotate(clockwise=False)
    time.sleep(3)