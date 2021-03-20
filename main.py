import random
import time
import types
from threading import Timer, Event

import RPi.GPIO as GPIO

import controller as con
import demo
from sixRUS import sixRUS

# GLOBALS
programStopped = Event()  # Event to set if program gets
shouldNotListen2Cont = Event()  # Event for checking if the program should listen to the Controller
robotMode = ''
joystick = None  # global varriable for joystick-class
alreadyConnected = False  # check if contoller reconnected


def call_every_5_sec():
    """execute everything in here every 5 seconds after"""
    if programStopped.is_set():  # only execute routine if program is not terminated
        return

    # Check if the controller is connected
    connected = con.stillConnected()

    global alreadyConnected

    if not connected:
        alreadyConnected = False
        print("Please connect controller! Retrying in 5 seconds...")
    else:
        if alreadyConnected:  # controller is still connected
            print('Controller still connected.')
            # no new initialisation required here
        else:
            stopListening2Cont()  # stop listening as the controller gets initalised 
            init_global_joystick()  # init new joystick since the controller was reconnected or connected the first time
            startListening2Cont()
            alreadyConnected = True
            print('Controller connected.')

    Timer(5.0, call_every_5_sec).start()  # call program again after 5 seconds


def call_every_tenth_sec():
    """call every 0.1 seconds"""
    if programStopped.is_set() or shouldNotListen2Cont.is_set():  # only execute routine if program is not terminated
        return

    global joystick
    controls = con.get_controller_inputs(joystick)

    # evaluate the answer from controller
    eval_controller_response(con.mode_from_inputs(controls))

    Timer(0.1, call_every_tenth_sec).start()  # call program again after 0.1 seconds


def init_global_joystick():
    """initalizes the controller as a global joystick varriable"""

    global joystick
    joystick = con.initCont()
    return joystick


def eval_controller_response(response):
    """evaluates the answer from the mode_from_input-function"""

    if isinstance(response, str):
        global robotMode

        if response == 'stop':
            pass
        elif response == 'homing':
            stopListening2Cont()
            pass
        elif response == 'demo':
            pass
        elif response == 'manual':
            pass
        elif response == 'calibrate':
            pass
        else:
            raise Exception("Unknown answer from controller")

        if robotMode != response:  # only print if the mode changes
            print('Switching to:', response)
            robotMode = response  # set robot mode to the response
            return True

    return False  # no response given


def startListening2Cont():
    """start listening to controller every 0.1 s"""
    global shouldNotListen2Cont
    shouldNotListen2Cont.clear()
    Timer(0.1, call_every_tenth_sec).start()


def stopListening2Cont():
    """stop listening to controller. Needed if a program is listening to 
    the controller itself or a new joystick gets initialised"""
    global shouldNotListen2Cont
    shouldNotListen2Cont.set()


def mov_with_controller(robot, dt=0.001):
    """This is the manual controlling mode, where the robot can be driven with the controller.
    Exits only if the mode was changed or the program was interrupted"""

    global joystick

    while True:  # infinite loop
        time.sleep(dt)

        inputs = con.get_controller_inputs(joystick)
        new_pose = con.get_movement_from_cont(inputs, robot.currPose)  # calc new pose

        quit = eval_controller_response(con.mode_from_inputs(inputs))  # check if mode was changed
        if quit:
            break

        robot.mov(new_pose)


def move_with_demo(robot):
    """Selects a random demo programm and executes it"""

    modules = list_of_modules(demo)
    prog = random.choice(modules)  # choose a random demo
    demo_pos_list = prog()  # execute chosen demo programm

    global robotMode

    for pos in demo_pos_list:
        try:
            if pos[6] == 'lin':
                coord = pos[:6]  # extract only pose
                robot.mov_lin(coord)  # move linear
            elif pos[6] == 'mov':
                coord = pos[:6]  # extract only pose
                robot.mov(coord)  # move with PTP-interplation
        except IndexError:
            robot.mov(pos)  # if 'lin' or 'mov' wasent given, use mov/PTP

        if not robotMode == 'demo':  # break if the mode was changed
            break


def list_of_modules(packageName):
    """Find all modules in a package 
    `packageName`: Name of package
    `return`: List of all modules in this package
    """
    modul_list = []
    for a in dir(demo):
        if isinstance(getattr(demo, a), types.FunctionType):
            modul_list.append(getattr(demo, a))

    return modul_list


def calibrate_process(robot, dt=0.005):
    """enters mode, where the user can calibrate each motor in microstep mode.
    A homing procedure has to be done afterwarts!
    
    `dt`: how fast the controller inputs get checked in [s]
    """

    global joystick
    mot_num = 0  # motornumber from 0 to 5
    # pose after calibration has to be given to move the motors but is not necessary here 
    # since a homing procedure has to be done afterwards anyways
    pose_after_cali = [0, 0, 0, 0, 0, 0]
    allowed_to_change_again = True  # if the next motor can be selected

    while True:
        time.sleep(dt)
        controls = con.get_controller_inputs(joystick)

        cali_mot = [0, 0, 0, 0, 0, 0]

        if allowed_to_change_again:
            # change motornumber with L1 and R1
            if controls['L1']:
                mot_num -= 1
            elif controls['R1']:
                mot_num += 1

            # check if selected motor number exists
            if mot_num > 5:
                mot_num = 0
            elif mot_num < 0:
                mot_num = 5
            allowed_to_change_again = False

        if controls['L1'] == 0 and controls['R1'] == 0:  # both buttons have to be released to switch to next motor
            allowed_to_change_again = True

        if controls['UP']:
            cali_mot[mot_num] = 1  # set 1 posivitve for selected motor
        elif controls['DOWN']:
            cali_mot[mot_num] = -1  # set -1 posivitve for selected motor

        robot.mov_steps(cali_mot, pose_after_cali)

        quit = eval_controller_response(con.mode_from_inputs(controls))  # check if mode was changed
        if quit:
            break


def main():
    global robotMode
    robotMode = 'demo'  # current mode (check documentation for all possible modes)

    robo = sixRUS(stepper_mode=1 / 32, step_delay=0.002)  # init robot

    robo.homing('90')  # home robot

    init_global_joystick()

    call_every_5_sec()  # call subroutine every 5-seconds to check for controller

    startListening2Cont()  # start listening to controller

    while True:  # infinite loop only breaks on Keyboard-Interrupt
        while robotMode == 'demo':
            move_with_demo(robo)
            time.sleep(2)  # wait and then execute the next function

        while robotMode == 'homing':
            stopListening2Cont()  # stop listening to controller to prevent program change while homing
            time.sleep(1.5)  # wait a bit to reduce multiple homing attempts
            robo.homing('90')  # use homing method '90'
            startListening2Cont()  # listen again
            robotMode = 'stop'  # exit homing

        while robotMode == 'manual':  # controll the robot with the controller
            stopListening2Cont()  # stop listening to controller (bc. we listen all the time in here)
            mov_with_controller(robo)
            startListening2Cont()  # let the program listen to the controller periodically again

        while robotMode == 'stop':  # stop robot after next movement and do nothing
            first_time = True
            while robotMode == 'stop':
                if first_time:
                    print("Stopped robot!")
                    first_time = False
                time.sleep(0.0001)  # limit loop time

        while robotMode == 'calibrate':
            stopListening2Cont()  # stop listening to controller (bc. we listen all the time in here)
            time.sleep(0.5)
            calibrate_process(robo)
            time.sleep(0.5)
            startListening2Cont()  # let the program listen to the controller periodically again
            robotMode = 'homing'  # home robot afterwards


# main program if this file get executed
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:  # shutdown python program gently
        print('Stopped with KeyboardInterrupt!')
    finally:
        GPIO.cleanup()  # cleanup GPIOs (to avoid warning on next startup)
        programStopped.set()  # set event for stopping threading
        # Exiting message
        print(
            "\n6-RUS program was terminated due to user-input or an error (Please wait ca. 5s) \nPlease start the program again to control the robot again!")
