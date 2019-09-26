#!/usr/bin/env python
import serial 
import time
import struct 

from Ranger import autoRange
#port = '/dev/ttyACM0' #port for seriel kontakt

class Arduino():
    def __init__(self, speedLimit = 1.0):
        self.speeds = [90,90,90,90,90]
        self.motor = 0
        self.ror = 0
        self.range_motor = autoRange(0.0,100.0,90.0,140.0) #180 for full speed
        self.range_ror = autoRange(-45.0,45.0,0.0,180.0)

        self.speed_limiter = speedLimit
        self.error_message = ""
        self.has_connection = False

    def connect(self, port = "/dev/ttyACM0", baud = 9800):
        try:
            self.serial_connection = serial.Serial(port, baud, timeout=1)
            print("Connection to " + port + " established succesfully!\n")
            self.has_connection = True
        except Exception as e:
            print(e)

    def neutral_start(self):
        self._setValues()
        try: 
            self.serial_connection.write(bytes(self.object_send))
            print("Boat started correctly")
            return True
        except Exception as error_message:
            self.error_message = error_message
            print(error_message)
    
    def update(self, motor, ror):
        ranged_motor = self.range_motor.new(motor)
        ranged_ror = self.range_ror.new(ror)

        ranged_motor = ranged_motor * self.speed_limiter
        self._setValues(ranged_motor, ranged_ror)
        try: 
            self.serial_connection.write(bytes(self.object_send))
        except Exception as error_message:
            self.error_message = error_message
            print(error_message)

    def _setValues(self, Motor_speed = 90, Rudder_angle = 90):
        self.object_send_temp = ''
        for i in range(3):
            self.speeds[i] = Motor_speed
        for j in range(3,5):
            self.speeds[j] = Rudder_angle
        for speed in self.speeds:
            self.object_send_temp += str(speed) + ':'
        self.object_send = self.object_send_temp + ';'
        self.object_send_temp = ''

    def is_ready(self):
        return self.has_connection 

    def get_error(self):
        return self.error_message




