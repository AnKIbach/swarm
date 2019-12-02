#!/usr/bin/env python
'''
This class is responsible for establishing a connection with an arduino through firmata

It also converts a vector to the propper signals to actuate the motor and servo connected to ports
5,6 9,10,11 on the arduino

Questions: anhellesnes@fhs.mil.no
'''
import time
import sys

import serial 
import pyfirmata 

from .Ranger import autoRange

class Arduino:
    '''Class resposible for compiling and sending data to arduino for motor and servo control'''

    def __init__(self, port, speedLimit = 1.0):
        '''Creates a new connection with ardunio

        Args:
            port: string of serial port wich arduino should connected to, typically /dev/ttyACMx
            speedLimit: integer that limits speed by procentage 0-100%
        '''

        self.motor_wanted  = 0.0
        self.rudder_wanted = 0.0
        self.port = port

        self.range_motor  = autoRange(0.0,5.0,90,180) 
        self.range_rudder = autoRange(-45.0, 45.0, 60, 120)

        self.speed_limt = speedLimit

        self.error = ''
        self.has_connection = False
        self.started_correctly = False

        try: 
            self.board = pyfirmata.Arduino(self.port)
            self.has_connection = True
        except serial.SerialException as e:
            self.error = e
            print("could not connect to arduino at", self.port, "with error: ", e)

        if self.has_connection == True: #defines list of pins to send data to
            self.pins = [self.board.get_pin('d:5:s'),
                        self.board.get_pin('d:6:s'),
                        self.board.get_pin('d:9:s'),
                        self.board.get_pin('d:10:s'),
                        self.board.get_pin('d:11:s')]

        try: 
            self._start()
        except serial.SerialException as e:
            self.error = e
            print("could not start boat with error: ", e)


    def __call__(self, motor = 0.0, rudder = 0.0):
        ''' call function to send new data to arduino
        
        Args:
            motor: float between 0.0 and max speed of boat in m/s, set to 0.0 as standard
            rudder: float between -45.0 and 45.0 in deg, set to 0.0 as standard
        '''
        self.rudder_wanted  = round(self.range_rudder.new(rudder), 3)
        self.motor_wanted   = round(self.range_motor.new(motor * self.speed_limt), 3)

        try:
            for i in range(0,1): #write for rudders
                self.pins[i].write(self.rudder_wanted)
            for i in range(2,5): 
                self.pins[i].write(self.motor_wanted)
        except pyfirmata.InvalidPinDefError as e:
            self.error = e
            print("could not set values with error: ", e)

    def _start(self):
        try:
            for i in range(5):
                self.pins[i].write(90)
            print("waiting for start...")
            time.sleep(4)
            self.started_correctly = True
        except serial.SerialException as e:
            self.error = e
            print("could not start motors correctly with error: ", e)
            #catch error and return true value for started

    def get_error(self):
        '''function to return error from arduino to user

        Returns:
            error message as string
        '''
        return self.error

    def get_current(self, what = 0):
        if what == 1:
            return self.motor_wanted 
        elif what == 2:
            return self.rudder_wanted
        elif what == 0:
            return [self.rudder_wanted, self.motor_wanted]
        else:
            print("no value to return selected...")

    def connection_state(self):
        '''Function to return connection state between RPi and Arduino

        Returns:
            connection state as boolean
        '''
        return self.has_connection

    def start_state(self):
        '''Function to return startup state from Arduino

        Returns:
            state as boolean
        '''
        return self.started_correctly

    def is_ready(self):
        '''Function to determine if arduino is ready to recieve data

        Returns:
            ready state as boolean
        '''
        if self.has_connection == True and self.started_correctly == True:
            return True
        else:
            return False 