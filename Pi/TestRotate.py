import time
import logging

from Controller import Controller

logger = logging.getLogger('Controller')

logger.addHandler(logging.StreamHandler())
logger.setLevel(logging.DEBUG)

controller = Controller()
while True:
    controller.rotate(True, 100)
    time.sleep(3)

    # controller.rotate(False, 100)
    # time.sleep(3)
