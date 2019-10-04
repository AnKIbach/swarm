
#!/usr/bin/env python
import time
import sys

import serial
import pyfirmata

def main():

    board = pyfirmata.Arduino('/dev/ttyACM0')

    # iter8 = pyfirmata.util.Iterator(board)
    # iter8.start()
    pin = board.get_pin('d:6:s')
    pin2 = board.get_pin('d:5:s')
    # pinmotor2 = board.get_pin('d:5:s')

    pin.write(90)
    pin2.write(90)
    time.sleep(2)

    while True:
        try:
            angle = float(input("enter angle: "))
            pin2.write(angle)

        except KeyboardInterrupt:
            sys.exit()

if __name__ == "__main__":
    main()