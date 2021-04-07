import RPi.GPIO as GPIO

from runtime import Runtime

# main program if this file get executed
if __name__ == '__main__':
    while True:
        app = Runtime()

        try:
            app.loop()
        except KeyboardInterrupt:  # shutdown python program gently
            print('Stopped with KeyboardInterrupt!')
            break
        finally:
            GPIO.cleanup()  # cleanup GPIOs (to avoid warning on next startup)
            app.program_stopped.set()
            # Exiting message
            print("6-RUS program was terminated due to user-input or an error (Please wait ca. 5s)")
            print("Please start the program again to control the robot again!")
