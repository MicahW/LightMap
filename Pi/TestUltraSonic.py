import time

from Controller import Controller

controller = Controller()
while True:
    value = controller.ultrasonic()
    print(value)

    time.sleep(1)
