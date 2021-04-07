import random
import time
import types
from threading import Event, Timer, RLock

import controller
import demo
from SixRUS import SixRUS


class Runtime:
    def __init__(self):
        # Thread-safe event flags
        self.program_stopped = Event()
        self.ignore_controller = Event()
        self._current_mode = 'off'
        self.controller = None
        self.already_connected = False
        self.controller_poll_rate = 5
        self.mode_poll_rate = 0.1
        self.robot = SixRUS(stepper_mode=1 / 32, step_delay=0.002)
        self.mode_lock = RLock()

    @property
    def current_mode(self):
        with self.mode_lock:
            return self._current_mode

    @current_mode.setter
    def current_mode(self, val):
        with self.mode_lock:
            self._current_mode = val

    def eval_controller_response(self, response):
        """
        evaluates the answer from the mode_from_input-function
        """
        if isinstance(response, str):
            if response in ['stop', 'demo', 'manual', 'calibrate', 'off']:
                pass
            elif response == 'homing':
                self.ignore_controller.set()
            else:
                raise ValueError("Unknown answer from controller")

            if self.current_mode != response:  # only print if the mode changes
                print(f'Switching from {self.current_mode} to {response}.')
                self.current_mode = response  # set robot mode to the response
                return True

        return False  # no response given

    def poll_controller_status(self):
        if self.program_stopped.is_set():
            return

        if not controller.still_connected():
            self.already_connected = False
            print("Please connect controller! Retrying in 5 seconds...")
        else:
            if self.already_connected:
                # no new initialisation required here
                print('Controller still connected.')
            else:
                # stop listening as the controller gets initalised
                self.ignore_controller.set()
                # init new joystick since the controller was reconnected or connected the first time
                self.controller = controller.init_controller()
                self.ignore_controller.clear()
                self.poll_program_mode()
                self.already_connected = True
                print('Controller connected.')

        # call program again after 5 seconds
        Timer(self.controller_poll_rate, self.poll_controller_status).start()

    def poll_program_mode(self):
        if self.program_stopped.is_set():
            # Finish thread if program is terminated
            return

        # Handle controller inputs all the time to keep button states up to date
        if controller.still_connected():
            controls = controller.get_controller_inputs(self.controller)
            if not self.ignore_controller.is_set():
                # evaluate the answer from controller
                self.eval_controller_response(controller.mode_from_inputs(controls))

        # call program again after 0.1 seconds
        Timer(self.mode_poll_rate, self.poll_program_mode).start()

    def move_manual(self, dt=0.005):
        """
        This is the manual controlling mode, where the robot can be driven with the controller.
        Exits only if the mode was changed or the program was interrupted
        """
        while True:
            if controller.still_connected():
                time.sleep(dt)
                inputs = controller.get_controller_inputs(self.controller)
                new_pose = controller.get_movement_from_cont(inputs, self.robot.currPose)

                # check if mode was changed
                if self.eval_controller_response(controller.mode_from_inputs(inputs)):
                    break
                self.robot.mov(new_pose)

    def move_demo(self):
        """
        Selects a random demo programm and executes it
        """
        modules = []
        for a in dir(demo):
            if isinstance(getattr(demo, a), types.FunctionType):
                modules.append(getattr(demo, a))

        prog = random.choice(modules)  # choose a random demo
        demo_pos_list = prog()  # execute chosen demo programm

        for pos in demo_pos_list:
            try:
                if pos[6] == 'lin':
                    coord = pos[:6]  # extract only pose
                    self.robot.mov_lin(coord)  # move linear
                elif pos[6] == 'mov':
                    coord = pos[:6]  # extract only pose
                    self.robot.mov(coord)  # move with PTP-interplation
            except IndexError:
                self.robot.mov(pos)  # if 'lin' or 'mov' wasent given, use mov/PTP

            if not self.current_mode == 'demo':  # break if the mode was changed
                break

    def calibrate_process(self, dt=0.005):
        """
        enters mode, where the user can calibrate each motor in microstep mode.
        A homing procedure has to be done afterwarts!

        `dt`: how fast the controller inputs get checked in [s]
        """
        mot_num = 0  # motornumber from 0 to 5
        # pose after calibration has to be given to move the motors but is not necessary here
        # since a homing procedure has to be done afterwards anyways
        pose_after_cali = [0, 0, 0, 0, 0, 0]
        allowed_to_change_again = True  # if the next motor can be selected
        cali_step_increment = 1

        while True:
            if controller.still_connected():
                time.sleep(dt)
                controls = controller.get_controller_inputs(self.controller)

                # check if mode was changed
                if self.eval_controller_response(controller.mode_from_inputs(controls)):
                    break

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
                    cali_mot[mot_num] = cali_step_increment  # set 1 posivitve for selected motor
                elif controls['DOWN']:
                    cali_mot[mot_num] = -cali_step_increment  # set -1 posivitve for selected motor
                elif controls['LEFT'] and self.robot.stepperMode != 0:
                    # Decrement calibration step size but needs to be at least 1
                    cali_step_increment = max(1, cali_step_increment - 1)
                elif controls['RIGHT'] and self.robot.stepperMode != 0:
                    # Increment calibration step size but needs to be <= 1/step-mode (e.g. 1/32 -> 32 inc = 1 full step)
                    cali_step_increment = min(1 / self.robot.stepperMode, cali_step_increment + 1)

                self.robot.mov_steps(cali_mot, pose_after_cali)

    def loop(self):
        self.robot.homing('90')  # home robot
        self.controller = controller.init_controller()
        # call subroutine every 5-seconds to check for controller
        self.poll_controller_status()
        # start listening to controller
        self.ignore_controller.clear()
        self.poll_program_mode()

        # infinite loop only breaks on Keyboard-Interrupt
        while True:
            # State Machine
            if self.current_mode == 'off':
                self.robot.disable_steppers()
                time.sleep(0.0001)  # limit loop time
            else:
                self.robot.enable_steppers()

                if self.current_mode == 'demo':
                    self.move_demo()
                    time.sleep(2)  # wait and then execute the next function

                elif self.current_mode == 'manual':
                    # control the robot with the controller
                    # stop listening to controller (bc. we listen all the time in here)
                    self.ignore_controller.set()
                    self.move_manual()
                    self.ignore_controller.clear()

                elif self.current_mode == 'calibrate':
                    # stop listening to controller (bc. we listen all the time in here)
                    self.ignore_controller.set()
                    time.sleep(0.5)
                    self.calibrate_process()
                    time.sleep(0.5)
                    # home robot afterwards
                    print('Switching to homing')
                    self.current_mode = 'homing'

                elif self.current_mode == 'homing':
                    # stop listening to controller to prevent program change while homing
                    self.ignore_controller.set()
                    time.sleep(1.5)  # wait a bit to reduce multiple homing attempts
                    self.robot.homing('90')  # use homing method '90'
                    # exit homing and switch to state that stopped calibration
                    print('Switching to stop')
                    self.current_mode = 'stop'
                    self.ignore_controller.clear()

                elif self.current_mode == 'stop':
                    # stop robot after next movement and do nothing
                    time.sleep(0.0001)  # limit loop time
