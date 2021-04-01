import pygame
import time
import os
from math import radians, degrees
from ps5_mapping import *


def initCont():
    """Inits controller to use it and returns joystick class.
    `returns` `None` if controller is not connected"""
    pygame.joystick.quit()
    pygame.init()
    pygame.joystick.init()

    try:  # check if controller is connected
        joystick = pygame.joystick.Joystick(0)  # assign the controller as joystick
        joystick.init()
    except Exception:  # not connected
        return None

    return joystick


def stillConnected():
    """Checks if a Controller is still connected trough a linux command.
    `returns` boolean"""

    print('Checking connection to controller:')
    controllerStatus = os.system('ls /dev/input/js0')  # checking for controller with linux

    if controllerStatus != 0:  # not connected
        return False
    else:
        return True


def get_movement_from_cont(controls, currentPose):
    """Calculates new pose from controller-input ans returns it as a list
    `controls`:dict  inputs from controller
    `currentPose`:list  poselist of current pose"""

    pose = currentPose
    pose = [pose[0], pose[1], pose[2], degrees(pose[3]), degrees(pose[4]), degrees(pose[5])]  # convert angles to DEG

    # 0Z---> y
    # |
    # V x 

    # speedfactors
    rotFac = 0.25  # Rotationspeed
    transFac = 1  # Translationspeed

    # add controller inputs to values
    pose[0] += controls['LS_UD'] * transFac
    pose[1] += controls['LS_LR'] * transFac
    pose[2] += controls['RS_UD'] * transFac * -1
    pose[3] += controls['LEFT'] * rotFac
    pose[3] -= controls['RIGHT'] * rotFac
    pose[4] += controls['DOWN'] * rotFac
    pose[4] -= controls['UP'] * rotFac
    pose[5] += controls['L1'] * rotFac
    pose[5] -= controls['R1'] * rotFac

    # move pose within workingspace if its outside
    pose = checkMaxVal(pose, 40, 40, [-150, -60], 40, 40, 30)  # this uses degrees

    pose = [pose[0], pose[1], pose[2], radians(pose[3]), radians(pose[4]), radians(pose[5])]  # convert to RAD

    return pose


def get_controller_inputs(joystick):
    """Gets all inputs from controller and returns them as a dict"""

    pygame.event.get()  # get event
    pygame.event.clear()  # clear events in queue (only one event needed)

    hat_x, hat_y = joystick.get_hat(LRUD_HAT)
    up = int(hat_y > 0)
    down = int(hat_y < 0)
    left = int(hat_x < 0)
    right = int(hat_x > 0)    
    
    input_values = {
        # buttons:
        'xBut': joystick.get_button(CROSS_BUTTON),
        'oBut': joystick.get_button(CIRCLE_BUTTON),
        'triangBut': joystick.get_button(TRIANGLE_BUTTON),
        'squareBut': joystick.get_button(SQUARE_BUTTON),
        # start/select/PS:
        'SELECT': joystick.get_button(SELECT_BUTTON),
        'START': joystick.get_button(START_BUTTON),
        'PS': joystick.get_button(PS_BUTTON),
        # control pad:
        'UP': up,
        'DOWN': down,
        'LEFT': left,
        'RIGHT': right,
        # trigger:
        'L1': joystick.get_button(L1_BUTTON),
        'R1': joystick.get_button(R1_BUTTON),
        'L2_': joystick.get_axis(L2_AXIS),  # as axis (no boolean)
        'R2_': joystick.get_axis(R2_AXIS),
        # joysticks:
        'LS_LR': joystick.get_axis(LS_LR_AXIS),  # LS = left stick
        'LS_UD': joystick.get_axis(LS_UD_AXIS),
        'LS': joystick.get_button(LS_BUTTON),
        'RS_LR': joystick.get_axis(RS_LR_AXIS),  # RS = right stick
        'RS_UD': joystick.get_axis(RS_UD_AXIS),
        'RS': joystick.get_button(RS_BUTTON),
    }

    input_values.update(
        {
            'L2': int(input_values['L2_'] != L2_PASSIVE_VAL),
            'R2': int(input_values['R2_'] != L1_PASSIVE_VAL)
        }
    )
    return input_values


def mode_from_inputs(inputs):
    """returns the selected mode from the controller inputs as a str. Returns `None` if no mode was choosen"""
    # Buttons on the right side
    xBut = inputs['xBut']
    oBut = inputs['oBut']
    triangBut = inputs['triangBut']
    squareBut = inputs['squareBut']

    # START and SELECT
    startBut = inputs['START']
    selectBut = inputs['SELECT']

    # R2 and L2
    R2But = inputs['R2']
    L2But = inputs['L2']

    R2andL2 = R2But and L2But

    # modes and returning of the mode as string
    if oBut == 1 and xBut == 0 and triangBut == 0 and squareBut == 0:
        return 'stop'
    elif xBut == 1 and triangBut == 0 and squareBut == 0 and oBut == 0 and R2andL2:  # do homing procedure
        return 'homing'
    elif startBut == 0 and selectBut == 1:  # start demo program
        return 'demo'
    elif startBut == 1 and selectBut == 0:  # change to manual control with controller
        return 'manual'
    elif xBut == 0 and triangBut == 1 and squareBut == 0 and oBut == 0 and R2andL2:  # calibrate motors
        return 'calibrate'

    return None


def checkMaxVal(val, maxX, maxY, zBounds, maxA, maxB, maxC):
    # Maximale x-Richtung
    val[0] = max(min(val[0], maxX), -maxX)

    # Maximale y-Richtung
    val[1] = max(min(val[1], maxY), -maxY)

    # Maximale z-Richtung
    val[2] = max(min(val[2], zBounds[1]), zBounds[0])

    # Maximale a-Richtung
    val[3] = max(min(val[3], maxA), -maxA)

    # Maximale b-Richtung
    val[4] = max(min(val[4], maxB), -maxB)

    # Maximale c-Richtung
    val[5] = max(min(val[5], maxC), -maxC)

    return val


# Example on how to use it in the main function
if __name__ == '__main__':
    joy = initCont()

    pose = [0, 0, 0, 0, 0, 0]  # define pose

    print(joy.get_name())
    print(f'Number of buttons: {joy.get_numbuttons()}')
    print(f'Number of axes: {joy.get_numaxes()}')
    print(f'Number of hats: {joy.get_numhats()}')
    time.sleep(2)

    while True:
        time.sleep(0.1)
        ans = get_controller_inputs(joy)

        print(ans)

