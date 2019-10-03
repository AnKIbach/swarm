
#!/usr/bin/env python
import time
import sys

import serial
import pyfirmata

def main():
    board = pyfirmata.Arduino('/dev/ttyACM1')
    pin = board.get_pin('d:5:s')

    try:
        angle = int(input("enter angle: "))
        pin.write(angle)

    except KeyboardInterrupt:
        sys.exit()

if __name__ == "__main__":
    main()