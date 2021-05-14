from sys import argv

import RPi.GPIO as GPIO
import logging
from runtime import Runtime

# main program if this file get executed
if __name__ == '__main__':
    logging.basicConfig(filename='log.txt', level=logging.DEBUG)

    try:
        robot_type = argv[1]
    except IndexError:
        logging.exception("Need to supply robot type as command-line argument")
        raise

    app = Runtime(robot_type)
    try:
        app_initialized = True
        app.loop()
    except KeyboardInterrupt:  # shutdown python program gently
        print("Stopped with KeyboardInterrupt!")
    except Exception as e:
        logging.exception(e)
    finally:
        GPIO.cleanup()  # cleanup GPIOs (to avoid warning on next startup)
        app.program_stopped.set()
        # Exiting message
        print("6-RUS program was terminated due to user-input or an error (Please wait ca. 5s)")
        print("Please start the program again to control the robot again!")
