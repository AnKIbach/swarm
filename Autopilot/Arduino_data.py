#!/usr/bin/env python
import time
import sys

import serial
import pyfirmata

from Ranger import autoRange

class Arduino:
    def __init__(self, speedLimit = 1.0, port = '/dev/ttyACM1'):
        self.motor_wanted  = 0.0
        self.rudder_wanted = 0.0
        self.port = port

        #rangers for converting speed to output
        self.range_motor  = autoRange(0.0,100.0,90.0,180.0)
        self.range_rudder = autoRange(-45.0, 45.0, 0.0, 180.0)

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

        self.pins = [self.board.get_pin('d:6:s'),
                    self.board.get_pin('d:5:s'),
                    self.board.get_pin('d:11:p'),
                    self.board.get_pin('d:10:p'),
                    self.board.get_pin('d:9:p')]

        try: 
            self._start()
        except serial.SerialException as e:
            self.error = e
            print"could not start boat with error: ", e)

        # self.rudder_left     = self.board.get_pin('d:6:s')
        # self.rudder_right    = self.board.get_pin('d:5:s')
        # self.motor_center    = self.board.get_pin('d:11:p')
        # self.motor_left      = self.board.get_pin('d:10:p')
        # self.motor_right     = self.board.get_pin('d:9:p')

    def __call__(self, motor = 50.0, rudder = 0.0):
        self.rudder_wanted  = self.range_rudder.new(rudder)
        self.motor_wanted   = self.range_motor.new(motor * self.speed_limt)
        try:
            for i in range(0,2): #write for rudders
                self.pins[i].write(self.rudder_wanted)
            for i in range(2,5): 
                self.pins[i].write(self.motor_wanted)
        except pyfirmata.InvalidPinDefError as e:
            self.error = e
            print("could not set values with error: ", e)

    def _start(self):
        try:
            for pin in self.pins:
                self.pins[pin].write(90.0)
            time.sleep(1)
            self.started_correctly = True
        except serial.SerialException as e:
            self.error = e
            print("could not start motors correctly with error: ", e)
            #catch error and return true value for started

    def get_error(self):
        return self.error

    def get_current(self, what = 0):
        if what == 1:
            return self.motor_wanted #add pin.read function if wanted
        elif what == 2:
            return self.rudder_wanted
        elif what == 0:
            return [self.rudder_wanted, self.motor_wanted]
        else:
            print("no value to return selected...")

    def connection_state(self):
        return self.has_connection

    def start_state(self):
        return self.started_correctly

    def is_ready(self):
        if self.has_connection == True and self.started_correctly == True:
            return True
        else:
            return False 