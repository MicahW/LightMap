
# Reads inputs from a gamepad and uses it to run the robot
# Provide number where /dev/input/event{number} is the file for reading gampad events from

import sys
import math
from evdev import InputDevice, categorize, ecodes

class GamePadController:
    # Event codes for gamepad inputs
    X_AXIS_CODE = 3
    Y_AXIS_CODE = 4

    # Threshold for setting read gampad positions as zero
    DEADZONE_THRESHOLD = 130

    def __init__(self, event_file_number):
        # Set up game pad object
        event_path = "/dev/input/event{}".format(event_file_number)
        self.gamepad = InputDevice(event_path)
        print(self.gamepad)


        # Set up current position (x, y) as centered
        self.position = [0, 0]

    def get_angle_and_magnitude(self, x_pos, y_pos):
        x = float(x_pos)
        y = float(y_pos)

        magnitude = math.sqrt((x * x) + (y * y))

        if y == 0:
            angle = 0.0
        else:
            angle = math.degrees(math.atan(x/ y))

        # Reverse angle if in quadrent 1 or 3
        if (x > 0 and y > 0) or (x < 0 and y < 0):
            angle = 90 - angle

        # Acount for quadrent 2
        if (x <= 0) and (y > 0):
            angle = abs(angle) + 90
        # Acount for quadrent 3
        elif(x < 0) and (y <= 0):
            angle = abs(angle) + 180
        # Acount for quadrent 4
        elif(x >= 0) and (y < 0):
            angle = abs(angle) + 270

        return angle, magnitude

    def send_motor_control(self):
        angle, magnitude = self.get_angle_and_magnitude(self.position[0], self.position[1])

    def run(self):
        for event in self.gamepad.read_loop():
            if (event.code == self.X_AXIS_CODE) or (event.code == self.Y_AXIS_CODE):
                value = event.value

                # The y axis reading is represented as negative max value for completly
                # foward, so reverse it here to fit standerd unit circle
                if (event.code == self.Y_AXIS_CODE):
                    value *= -1

                # If the position is within the DEADZONE value then set it to zero
                if (abs(value) <= self.DEADZONE_THRESHOLD):
                    value = 0

                # Update the position
                position_index = 0 if event.code == self.X_AXIS_CODE else 1
                self.position[position_index] = value

                # Send the motor control message for the updated value
                self.send_motor_control()

# If running as main
controller = GamePadController(sys.argv[1])
controller.run()