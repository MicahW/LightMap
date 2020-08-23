# -*- coding: utf-8 -*-

import serial
import struct
import logging

logger = logging.getLogger('Controller')


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

    MOVE_STRAIGHT_REQUEST_ID = 0x07

    STOP_REQUEST_ID = 0x08

    ROTATE_REQUEST_ID = 0x09
    ROTATE_RESPONSE_ID = 0x0A

    def __init__(self, device_name=DEFAULT_DEVICE_NAME):
        '''
        Initialize Controller object and open serial device for read/write.
        '''

        self.device_name = device_name
        self.device = serial.Serial(device_name)

    def servo(self, degrees=0):
        ''' Set servo position '''

        logger.debug(f'servo: enter: degrees={degrees}')

        assert (abs(degrees) <= self.SERVO_MAX_DEGREES)
        self.transmit(struct.pack('Bb', self.SERVO_CONTROL_ID, degrees))

        logger.debug('servo: exit')

    def move_straight(self):
        ''' Move the robot straight untill a motor stop message is sent '''

        logger.debug('move_straight: enter')

        self.transmit(struct.pack('B', self.MOVE_STRAIGHT_REQUEST_ID))

        logger.debug('move_straight: exit')

    def stop(self):
        ''' Request that the robot stop moving '''

        logger.debug('stop: enter')

        self.transmit(struct.pack('B', self.STOP_REQUEST_ID))

        logger.debug('stop: exit')

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

        logger.debug(f'motor: enter: right_forward={right_forward}, '
                     f'right_speed={right_speed}, '
                     f'left_forward={left_forward}, '
                     f'left_speed={left_speed}')

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

        logger.debug('ultrasonic: enter')

        # Send the request
        self.transmit(struct.pack('B', self.ULTRASONIC_REQUEST_CONTROL_ID))

        # Read the response
        response = self.receive(5)

        control_byte, value = struct.unpack('<BI', response)

        assert control_byte == self.ULTRASONIC_RESPONSE_CONTROL_ID
        assert 0 <= value and value < 2**32

        logger.debug(f'ultrasonic: exit: value={value}')

        return value

    def ping(self):
        ''' Send ping to ensure remote connection. '''

        logger.debug('ping: enter')

        # Send the request
        self.transmit(struct.pack('B', self.PING_REQUEST_CONTROL_ID))

        # Receeive the response
        response = self.receive(1)

        control_byte = struct.unpack('B', response)[0]

        assert control_byte == self.PING_RESPONSE_CONTROL_ID

        logger.debug('ping: exit')

    def rotate(self, clockwise, steps):
        '''
        Send a request to rotate, clockwise (True) or counter
        clockwise (False) a specified amount of steps
        '''

        logger.debug(f'rotate: enter: clockwise={clockwise} steps={steps}')

        self.transmit(struct.pack('BBB', self.ROTATE_REQUEST_ID,
                                  steps,
                                  (0 if clockwise else 1)))

        response_id = struct.unpack("B", self.receive(1))[0]
        assert response_id == self.ROTATE_RESPONSE_ID

        logger.debug('rotate: exit')

    def transmit(self, data):
        ''' Transmit bytes to the serial device. '''

        logger.debug(f'transmit: write {len(data)} bytes: {data}')

        self.device.write(data)

    def receive(self, n):
        ''' Receive n bytes from the serial device '''
        logger.debug(f'receive: read {n} bytes')

        data = self.device.read(n)

        logger.debug(f'receive: {data}')

        return data
