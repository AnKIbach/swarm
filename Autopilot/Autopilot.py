#!/usr/bin/env python
import time
import sys

from Autopilot_callFunc import Autopilot
from To_arduino import Arduino
from GPS_definition import GPS
from Vector_definition import Vector
from ROS_operators.Navigation_data import navData


def main():
    nav = navData()
    autopilot = Autopilot()
    arduino = Arduino()
    arduino.connect()
    arduino.neutral_start()

    wanted_GPS = GPS(60.394238, 5.266250)

    wait_time = 0.0

    while nav.get_connection_state() == False or arduino.is_ready() == False:
    #waits for both systems to connect
        wait_time += 0.1
        time.sleep(0.1)
        if wait_time > 10.0: #exit if timeout is over 10s
            sys.exit(0)

    try:
        while True:
            current_GPS = nav.get_GPS()
            current_vector = nav.get_Vector()

            wanted_vector = wanted_GPS.calculate(current_GPS)

            autopilot.set_wanted_vector(wanted_vector)

            speed = input("enter speed: ")
            current_vector.magnitude = speed
            change = autopilot(current_vector)

            arduino.update(change.magnitude, change.angle)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("exiting...")
        sys.exit(0)

if __name__ == "__main__":
    main()

