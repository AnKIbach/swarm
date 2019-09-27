import time
import sys

import pyfirmata

from Arduino_data import Arduino

def main():
    board = pyfirmata.Arduino('/dev/ttyACM0')
    ror =  board.get_pin('d:6:s')
    ror2 =  board.get_pin('d:5:s')
    motor = board.get_pin('d:11:p')
    motor2 = board.get_pin('d:10:p')
    motor3 = board.get_pin('d:9:p')

    try:
        while True:
            speed = int(input("enter speed: "))
            ror.write(speed)
            ror2.write(speed)
            motor.write(speed)
            motor2.write(speed)
            motor3.write(speed)
    except ValueError:
        ror.write(90)
        motor.write(90)
    except KeyboardInterrupt:
        ror.write(90)
        motor.write(90)


if __name__ == "__main__":
    main()
