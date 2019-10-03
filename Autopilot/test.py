
#!/usr/bin/env python
import time
import sys

import serial
import pyfirmata

def main():

    board = pyfirmata.Arduino('/dev/ttyACM1')

    # iter8 = pyfirmata.util.Iterator(board)
    # iter8.start()
    pin = board.get_pin('d:9:s')
    while True:
        try:
            angle = float(input("enter angle: "))
            pin.write(angle)

        except KeyboardInterrupt:
            sys.exit()

if __name__ == "__main__":
    main()