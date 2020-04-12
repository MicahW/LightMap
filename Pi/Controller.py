# -*- coding: utf-8 -*-

import serial
import struct


class Controller:

    DEFAULT_DEVICE_NAME = '/dev/serial0'

    MOTOR_CONTROL_ID = 0x01
    MOTOR_CONTROL_FORWARD = 0x00
    MOTOR_CONTROL_BACKWARD = 0x80

    SERVO_CONTROL_ID = 0x02
    SERVO_MAX_DEGREES = 80

    ULTRASONIC_REQUEST_CONTROL_ID = 0x03
    ULTRASONIC_RESPONSE_CONTROL_ID = 0x04

    PING_REQUEST_CONTROL_ID = 0x05
    PING_RESPONSE_CONTROL_ID = 0x06

    def __init__(self, device_name=DEFAULT_DEVICE_NAME):
        '''
        Initialize Controller object and open serial device for read/write.
        '''

        self.device_name = device_name
        self.device = serial.Serial(device_name)

    def servo(self, degrees=0):
        ''' Set servo position '''
        assert (abs(degrees) <= self.SERVO_MAX_DEGREES)
        self.transmit(struct.pack('Bb', self.SERVO_CONTROL_ID, degrees))

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

    def ultrasonic(self):
        '''
        Request current value from the ultra sonic sensor.  Returned value
        is a 4-byte unsigned int.
        '''

        # Send the request
        self.transmit(struct.pack('B', self.ULTRASONIC_REQUEST_CONTROL_ID))

        # Read the response
        response = self.receive(5)

        control_byte, value = struct.unpack('<BI', response)

        assert control_byte == self.ULTRASONIC_RESPONSE_CONTROL_ID
        assert 0 <= value and value < 2**32

        return value

    def ping(self):
        ''' Send ping to ensure remote connection. '''

        # Send the request
        self.transmit(struct.pack('B', self.PING_REQUEST_CONTROL_ID))

        # Receeive the response
        response = self.receive(1)

        (control_byte, ) = struct.unpack('B', response)

        assert control_byte == self.PING_RESPONSE_CONTROL_ID

    def transmit(self, data):
        ''' Transmit bytes to the serial device. '''

        self.device.write(data)

    def receive(self, n):
        ''' Receive n bytes from the serial device '''

        return self.device.read(n)
