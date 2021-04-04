import os
import time
from math import degrees as deg
from math import radians as rad
from typing import Dict

import pygame

from ps5_mapping import *


def init_controller():
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
    else:
        return joystick


def still_connected():
    """Checks if a Controller is still connected trough a linux command.
    `returns` boolean"""
    print('Checking connection to controller:')
    return os.system('ls /dev/input/js0') == 0


def get_movement_from_cont(controls, curr_pose):
    """Calculates new pose from controller-input ans returns it as a list
    `controls`:dict  inputs from controller
    `currentPose`:list  poselist of current pose"""
    pose = [curr_pose[0], curr_pose[1], curr_pose[2], deg(curr_pose[3]), deg(curr_pose[4]), deg(curr_pose[5])]

    # 0Z---> y
    # |
    # V x 

    # speedfactors
    rot_fac = 0.25
    trans_fac = 1

    # add controller inputs to values
    pose[0] += controls['LS_UD'] * trans_fac
    pose[1] += controls['LS_LR'] * trans_fac
    pose[2] += controls['RS_UD'] * trans_fac * -1
    pose[3] += (controls['LEFT'] - controls['RIGHT']) * rot_fac
    pose[4] += (controls['DOWN'] - controls['UP']) * rot_fac
    pose[5] += (controls['L1'] - controls['R1']) * rot_fac

    # move pose within workingspace if its outside
    pose = check_max_val(pose, 40, 40, [-150, -60], 40, 40, 30)  # this uses degrees
    pose = [pose[0], pose[1], pose[2], rad(pose[3]), rad(pose[4]), rad(pose[5])]  # convert to RAD

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


def mode_from_inputs(inputs: Dict[str, float]):
    """returns the selected mode from the controller inputs as a str. Returns `None` if no mode was choosen"""
    x = inputs['xBut']
    o = inputs['oBut']
    triangle = inputs['triangBut']
    square = inputs['squareBut']
    start = inputs['START']
    select = inputs['SELECT']
    r2 = inputs['R2']
    l2 = inputs['L2']

    # modes and returning of the mode as string
    if o and not x and not triangle and not square:
        return 'stop'
    elif select and not start:
        return 'demo'
    elif start and not select:
        return 'manual'
    elif r2 and l2:
        if triangle and not x and not square and not o:
            return 'calibrate'
        elif x and not triangle and not square and not o:
            return 'homing'

    return None


def check_max_val(val, max_x: float, max_y: float, z_bounds, max_a: float, max_b: float, max_c: float):
    # Maximum translation
    val[0] = max(min(val[0], max_x), -max_x)
    val[1] = max(min(val[1], max_y), -max_y)
    val[2] = max(min(val[2], z_bounds[1]), z_bounds[0])

    # Maximum angles
    val[3] = max(min(val[3], max_a), -max_a)
    val[4] = max(min(val[4], max_b), -max_b)
    val[5] = max(min(val[5], max_c), -max_c)

    return val


# Example on how to use it in the main function
if __name__ == '__main__':
    joy = init_controller()
    print(f'Controller name: {joy.get_name()}')
    print(f'Number of buttons: {joy.get_numbuttons()}')
    print(f'Number of axes: {joy.get_numaxes()}')
    print(f'Number of hats: {joy.get_numhats()}')
    time.sleep(2)

    while True:
        time.sleep(0.1)
        ans = get_controller_inputs(joy)
        print(ans)
