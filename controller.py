import pygame
import time
import os
from math import radians, degrees
from ps5_mapping import *


def init_cont():
    """Inits controller to use it and returns joystick class.
    `returns` `None` if controller is not connected"""
    pygame.joystick.quit()
    pygame.init()
    pygame.joystick.init()

    try:
        # check if controller is connected
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
    controller_status = os.system('ls /dev/input/js0')  # checking for controller with linux
    return controller_status == 0


def get_movement_from_cont(controls, current_pose):
    """Calculates new pose from controller-input ans returns it as a list
    `controls`:dict  inputs from controller
    `currentPose`:list  poselist of current pose"""

    # Convert angles to degrees
    current_pose[3] = degrees(current_pose[3])
    current_pose[4] = degrees(current_pose[4])
    current_pose[5] = degrees(current_pose[5])

    # 0Z---> y
    # |
    # V x 

    # speedfactors
    rot_fac = 0.25  # Rotationspeed
    trans_fac = 1  # Translationspeed

    # add controller inputs to values
    current_pose[0] += controls['LS_UD'] * trans_fac
    current_pose[1] += controls['LS_LR'] * trans_fac
    current_pose[2] -= controls['RS_UD'] * trans_fac
    current_pose[3] += controls['LEFT'] * rot_fac
    current_pose[3] -= controls['RIGHT'] * rot_fac
    current_pose[4] += controls['DOWN'] * rot_fac
    current_pose[4] -= controls['UP'] * rot_fac
    current_pose[5] += controls['L1'] * rot_fac
    current_pose[5] -= controls['R1'] * rot_fac

    # move pose within workingspace if its outside
    current_pose = check_max_val(current_pose, 40, 40, [-150, -60], 40, 40, 30)  # this uses degrees

    # convert to RAD
    current_pose[3] = radians(current_pose[3])
    current_pose[4] = radians(current_pose[4])
    current_pose[5] = radians(current_pose[5])

    return current_pose


def get_controller_inputs(joystick):
    """Gets all inputs from controller and returns them as a dict"""

    pygame.event.get()  # get event
    pygame.event.clear()  # clear events in queue (only one event needed)

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
        'UP': joystick.get_button(UP_BUTTON),
        'DOWN': joystick.get_button(DOWN_BUTTON),
        'LEFT': joystick.get_button(LEFT_BUTTON),
        'RIGHT': joystick.get_button(RIGHT_BUTTON),
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
    x_but = inputs['xBut']
    o_but = inputs['oBut']
    triang_but = inputs['triangBut']
    square_but = inputs['squareBut']

    # START and SELECT
    start_but = inputs['START']
    select_but = inputs['SELECT']

    # R2 and L2
    r2_but = inputs['R2']
    l2_but = inputs['L2']
    r2_and_l2 = r2_but and l2_but

    # modes and returning of the mode as string
    if o_but == 1 and x_but == 0 and triang_but == 0 and square_but == 0:
        return 'stop'
    elif x_but == 1 and triang_but == 0 and square_but == 0 and o_but == 0 and r2_and_l2:
        # do homing procedure
        return 'homing'
    elif start_but == 0 and select_but == 1:
        # start demo program
        return 'demo'
    elif start_but == 1 and select_but == 0:
        # change to manual control with controller
        return 'manual'
    elif x_but == 0 and triang_but == 1 and square_but == 0 and o_but == 0 and r2_and_l2:
        # calibrate motors
        return 'calibrate'

    return None


def check_max_val(val, max_x, max_y, z_bounds, max_a, max_b, max_c):
    """
    Limit the axes to their maximum values
    """
    val[0] = max(min(val[0], max_x), -max_x)
    val[1] = max(min(val[1], max_y), -max_y)
    val[2] = max(min(val[2], z_bounds[1]), z_bounds[0])
    val[3] = max(min(val[3], max_a), -max_a)
    val[4] = max(min(val[4], max_b), -max_b)
    val[5] = max(min(val[5], max_c), -max_c)
    return val


# Example on how to use it in the main function
if __name__ == '__main__':
    joy = init_cont()

    pose = [0, 0, 0, 0, 0, 0]  # define pose

    print(joy.get_name())
    print(joy.get_numbuttons())

    while True:
        time.sleep(0.1)
        ans = get_controller_inputs(joy)

        print(ans)
