#!/usr/bin/env python

from Vector_class import Vector
from Arduino_data import Arduino

def main():
    arduino = Arduino(speedLimit = 0.5) #speed limiter for testing
    arduino.connect()
    arduino.neutral_start()

if __name__ == "__name__":
    main()