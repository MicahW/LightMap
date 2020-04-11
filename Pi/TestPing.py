import time

from Controller import Controller

controller = Controller()
while True:
    controller.ping()
    print('ping!')

    time.sleep(1)