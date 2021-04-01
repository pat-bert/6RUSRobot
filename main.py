from threading import Event

import RPi.GPIO as GPIO

# GLOBALS
from runtime import Runtime

programStopped = Event()  # Event to set if program gets
shouldNotListen2Cont = Event()  # Event for checking if the program should listen to the Controller
robotMode = ''
joystick = None  # global varriable for joystick-class
alreadyConnected = False  # check if contoller reconnected

# main program if this file get executed
if __name__ == '__main__':
    app = Runtime()

    try:
        app.loop()
    except KeyboardInterrupt:  # shutdown python program gently
        print('Stopped with KeyboardInterrupt!')
    finally:
        GPIO.cleanup()  # cleanup GPIOs (to avoid warning on next startup)
        app.program_stopped.set()
        # Exiting message
        print("6-RUS program was terminated due to user-input or an error (Please wait ca. 5s)")
        print("Please start the program again to control the robot again!")
