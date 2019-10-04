#!/usr/bin/env python
import sys

from Vector_class import Vector
from Arduino_data import Arduino

def main():
    arduino = Arduino('dev/ttyACM0', speedLimit = 0.5) #speed limiter for testing
    if arduino.is_ready() == True:
        print("Boat reset, exiting...")
        sys.exit()

if __name__ == "__name__":
    main()