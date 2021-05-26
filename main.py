import logging
import os
from sys import argv

import RPi.GPIO as GPIO
from button import EXIT_SHUTDOWN
from runtime import Runtime

# main program if this file get executed
if __name__ == '__main__':
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)-15s %(threadName)-15s %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s'
    )

    try:
        robot_type = argv[1]
    except IndexError:
        logging.exception("Need to supply robot type as command-line argument")
        raise

    app = Runtime(robot_type)
    exit_code = None

    try:
        app_initialized = True
        app.loop()
    except KeyboardInterrupt:
        # shutdown python program gently
        logging.exception("Stopped with KeyboardInterrupt!")
    except SystemExit as e:
        # shutdown via button
        logging.exception("Stopped via button press")
        app.program_stopped.set()
        app.lcd.print_status('Shutting down...')
        exit_code = e.code
    except Exception as e:
        logging.exception(e)
    finally:
        app.program_stopped.set()
        # cleanup GPIOs (to avoid warning on next startup)
        GPIO.cleanup()

        if exit_code == EXIT_SHUTDOWN:
            os.system("sudo shutdown now")

        # Exiting message
        logging.info("6-RUS program was terminated due to user-input or an error (Please wait ca. 5s)")
        logging.info("Please start the program again to control the robot again!")
