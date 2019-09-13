#!/usr/bin/env python
import time

from To_arduino import Arduino
from PID import PID
from GPS_definition import GPS
from Vector_definition import Vector
from ROS_operators.Navigation_data import navData


def main():
    nav = navData()
    controller = PID()
    arduino = Arduino()
    arduino.connect()
    arduino.neutral_start()

    wantGPS = GPS(60.394238, 5.266250)


    while True:
        
        curGPS = nav.get_GPS()
        currentVect = nav.get_Vector()
        wantedVec = wantGPS.calculate(curGPS)
        wantedVec.magnitude = 0.0

        controller.set_wanted(wantedVec)
        out = controller.update(currentVect)
        outfart = currentVect.magnitude + out.magnitude
        arduino.update(outfart, out.angle)

if __name__ == "__main__":
    main()

