import time
import logging

from Controller import Controller

logger = logging.getLogger('Controller')

logger.addHandler(logging.StreamHandler())
logger.setLevel(logging.DEBUG)

controller = Controller()
while True:
    controller.ping()
    print('ping!')

    time.sleep(1)