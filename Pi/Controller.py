# -*- coding: utf-8 -*-

import serial
import struct


class Controller:

    DEFAULT_DEVICE_NAME = '/dev/serial0'

    MOTOR_CONTROL_ID = 0x01
    MOTOR_CONTROL_FORWARD = 0x00
    MOTOR_CONTROL_BACKWARD = 0x80

    def __init__(self, device_name=DEFAULT_DEVICE_NAME):
        '''
        Initialize Controller object and open serial device for read/write.
        '''

        self.device_name = device_name
        self.device = serial.Serial(device_name)

    def motor(self, right_forward=True, right_speed=0, left_forward=True,
              left_speed=0):
        ''' Motor control.
        right_forward, left_forward: 'forward' if True else 'backward'
        right_speed, left_speed: 0 <= n < 128
        '''

        assert isinstance(right_forward, bool)
        assert 0 <= right_speed and right_speed < 128

        assert isinstance(left_forward, bool)
        assert 0 <= left_speed and left_speed < 128

        control_byte = self.MOTOR_CONTROL_ID

        right_byte = self.MOTOR_CONTROL_FORWARD if right_forward \
            else self.MOTOR_CONTROL_BACKWARD
        right_byte |= right_speed

        left_byte = self.MOTOR_CONTROL_FORWARD if left_forward \
            else self.MOTOR_CONTROL_BACKWARD
        left_byte |= left_speed

        self.transmit(struct.pack('BBB', control_byte, right_byte, left_byte))

    def transmit(self, bytes):
        ''' Transmit bytes to the serial device. '''

        self.device.write(bytes)