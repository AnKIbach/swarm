import time
import sys

from To_arduino import Arduino

def main():
    arduino = Arduino()

    arduino.connect()

    arduino.neutral_start()

    try: 
        while True:
            sped = input("enter speed: ")
            ang = input("enter angle: ")
            arduino.update(sped, ang)

    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    main()
