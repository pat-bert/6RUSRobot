import math as m
from math import sin, cos
import numpy as np
import time
import RPi.GPIO as GPIO
import stepper

from scipy.optimize import fsolve
from slerp import slerp_pose, angle_to_turn


class sixRUS:
    """Class for the 6-RUS-robot"""

    def init_vars(self):
        """Calling this funciton in __init__ to make all variables dynamic"""

        self.currPose = [0.0]*6  # current pose of the robot: [x, y, z, alpha, beta, gamma]
        self.currSteps = [0]*6  # current motorangles as steps #TODO: this varriable is not updated yet

        # TODO: step variable to calculate the exact position

        self.stepsPerRev = 0.0  # how many steps the motor takes for one revolution (Full Step) (set in __init__)
        self.stepperMode = 0.0  # microstep Mode (set in __init__)

        self.stepAngle = 0.0  # Angle per step [rad] (calculated in __init__ function)
        self.stepDelay = 0.0  # Delay between steps (calculated in __init__ function)
        
        # Robot-Dimensions [mm]  (Hardcoded but can be changed via a function)
        # [l1, l2, dx, dy, Dx, Dy]  (more Infos in documentation) 
        # self.geometricParams = [57.0, 92.0, 11.0, 9.5, 63.0, 12.0]  # small endeffector
        self.geometricParams = [57.0, 92.0, 29.5, 12.5, 63.0, 12.0]  # big endeffector
        
        # Robot GPIO-pins:
        # Stepsize pins
        self.M0 = 21
        self.M1 = 20
        self.M2 = 16

        # Direction pins ([0] is Motor 1, [1] is Motor 2 and so forth)
        self.dirPins = [13, 5, 9, 22, 17, 3]

        # Step pins ([0] is Motor 1, [1] is Motor 2 and so forth)
        self.stepPins = [6, 11, 10, 27, 4, 2]

    def __init__(self, stepper_mode=1 / 32, steps_per_rev=200, step_delay=0.0208):
        """Initialise the Robot
        `stepperMode`: float  Microstepmode e.g. 1/32, 1/16, 1/8; 1/4, 1/2 or 1
        `stepsPerRev`: int  How many Full-steps the motors have
        `stepDelay`: in [s]   SleepDelay between steps lower time allows for faster rotation but is
        more susceptible of missing steps
        """
        self.init_vars()  # set all Varriables
        self.stepAngle = stepper_mode * 2 * m.pi / steps_per_rev  # angle corresponding to one step
        self.stepperMode = stepper_mode  # set mode to class variable
        self.stepDelay = stepper_mode * step_delay  # calculate time between steps
        self.init_gpio()  # initialise needed GPIO-pins

    def init_gpio(self):
        """This initialises all GPIO pins of the Raspberry Pi that are needed.  
        The pins are hardcoded and defined in the documentation! If they have to be 
        changed, edit the corresponting variables in this class"""

        GPIO.setmode(GPIO.BCM)  # use GPIO numbers (NOT pin numbers)

        # set up microstep pins
        mode = (self.M0, self.M1, self.M2)   # microstep resolution GPIO pins
        GPIO.setup(mode, GPIO.OUT)  # set M-pins (M0,M1,M2) as outputpins

        # dictionary for microstep mode
        resolution = {
                    1: (0, 0, 0),
                    1/2: (1, 0, 0),
                    1/4: (0, 1, 0),
                    1/8: (1, 1, 0),
                    1/16: (0, 0, 1),
                    1/32: (1, 0, 1)
                    }

        GPIO.output(mode, resolution[self.stepperMode])  # set Mode-Pins to desired values

        # init Step-Pins as output-pins
        for i in self.stepPins:
            GPIO.setup(i, GPIO.OUT)
            GPIO.output(i, GPIO.LOW)

        # init Direction-Pins as output-pins
        for i in self.dirPins:
            GPIO.setup(i, GPIO.OUT)
            GPIO.output(i, GPIO.LOW)

        # set !ENABLE-Pin Low
        GPIO.setup(0, GPIO.OUT)
        GPIO.output(0, GPIO.LOW)

    def angles2steps(self, angles: list):
        """converts list of angles [rad] to list of steps"""
        steps = [round(x / self.stepAngle) for x in angles]
        return steps

    # MOVING
    def mov_steps(self, step_list, new_pose: list):
        """ This funktion moves every motor x steps, where x are the number of steps to take 
        `stepList` is a np-array or list with 6 Values for the steps to take
        `newPose`:list is the pose after the movement was done
        """
        # movVec = np.array([2, -5, 1, -10, 0, 0])
        mov_vec = np.array(step_list)  # convert to np.array for vector calculations

        # compensate for motor placement (switch direction every second motor)
        rotation_compensation = np.array([1, -1, 1, -1, 1, -1])
        mov_vec = np.multiply(mov_vec, rotation_compensation)

        step_count = np.zeros(6, dtype=int)  # step counter for calculating on wich loop to move

        max_steps = int(max(abs(mov_vec)))  # maximum steps to move
        step_after_inc = max_steps / (abs(mov_vec) + 1)  # after which increments to take one step

        step_motors = [0]*6  # saves which motor to tun on each step (1 -> turn; 0 -> do not turn)

        # determine direction from sign of vector-element
        directions = [0]*6  # saves in which directions the motors should turn
        for i, _ in enumerate(directions):
            if mov_vec[i] < 0:
                directions[i] = 0
            else:
                directions[i] = 1
    
        for i in range(max_steps):  # loop with step for highest amount of steps
            
            step_motors = [0]*6  # reset

            for n, incNr in enumerate(step_after_inc):  # loop trough all motor step-values
                # n in here is the number of the targetmotor. Starting from 0
                
                if np.isinf(incNr):  # if number is infinite skip loop (happens if steps to move are 0)
                    continue
                
                c = round(((step_count[n] + 1) * incNr))

                if c == i or incNr < 1:  # test if a step should be executed
                    
                    step_count[n] += 1  # Adding steps for calculating the next step
                    step_motors[n] = 1  # Add that this motor should

            # Execute steps for motors, if they should step
            stepper.do_multi_step(step_motors, self.stepPins, self.dirPins, directions, delay=self.stepDelay)

        # Update current pose and current steps
        self.currSteps = list(np.array(self.currSteps) + np.array(step_list))
        self.currPose = new_pose

    def mov(self, pose: list):
        """Move to new position/pose with Point-to-Point (PTP) interpolation.
        This is a synchronous PTP implementation"""
        
        new_angles = self.inv_kinematic(pose)  # get new angles
        new_steps = self.angles2steps(new_angles)  # calculate steps of new position

        # create list of steps to move
        steps_to_move = np.array(new_steps) - np.array(self.currSteps)

        # move motors corresponding to stepsToMove-list
        self.mov_steps(steps_to_move, pose)

    def mov_lin(self, pose: list, pos_res: float = 10, ang_res: float = 3, vel: float = None):
        """
        Move to new position with linear interpolation  
        `pose`: list with values of the pose to move to  
        `posRes`: how many interpolating points should be used in [steps in cm]
        `angRes`: how many interpolating points should be used in [steps in (10*deg)]
        `vel`: how fast the robot should move [cm/s] (default is as fast as possible)
        """
        # Calculate distance to move
        x_dir = pose[0] - self.currPose[0]
        y_dir = pose[1] - self.currPose[1]
        z_dir = pose[2] - self.currPose[2]

        distance = m.sqrt(x_dir**2 + y_dir**2 + z_dir**2)  # distance to move [mm]

        steps_pos = distance * pos_res / 10  # Number of steps to move (calculated by distance)

        # Calculate angle to move
        angle_to_turn_val = angle_to_turn(self.currPose, pose)

        steps_rot = m.degrees(angle_to_turn_val) * ang_res / 10  # Number of steps to move (calculated by angle)

        # take the maximum steps needed to match resolution
        nr_of_steps = m.ceil(max([steps_pos, steps_rot]))

        if nr_of_steps <= 0:
            return  # return if poses are already identical

        # check if velocity was given
        if vel is not None:
            if vel > 0:
                # Calculate the timing for velocity management  
                t_ges = (distance/10) / vel  # calculate duration of whole movement
                dt_ideal = t_ges / nr_of_steps  # calculate time it should take to execute one loop iteration
            else:
                print('Given velocity is lower than 0 or 0! Using default!')

        poses = slerp_pose(self.currPose, pose, nr_of_steps + 1)  # calculate poses in between

        # initialization of variables needed in the loop
        t_curr = 0
        t_ideal = 0
        dt = 0

        t_st = time.time()  # check the time

        for i, poseBetw in enumerate(poses):

            self.mov(poseBetw)  # go to next pose
            self.currPose = poseBetw  # update Pose

            # Velocity management
            if vel is not None:  # if velocity is given make calculations for velocity-management
                t_ideal = t_st + (i+1) * dt_ideal  # calculate ideal time (to compare to)
                t_curr = time.time()  # get current time
                dt = t_ideal - t_curr  # copmarison between ideal and real timing

                if dt > self.stepDelay:  # if ideal timing is in front of the real time
                    time.sleep(dt)  # sleep until timings match again
                    
                elif dt < -2*self.stepDelay:  # if real time is too much behind
                    # TODO: Maybe turn on LED or something (just printing on console in this case)
                    print('Can not keep velocity!')

        return

    def homing(self, method: str):
        """Homing of the Robot
        `method`:str  chooses the method of homing

        Methods:
        ------ 
        '90': All arms connected to a motor point downwards  
        """

        if method == '90':
            # Homing with Homeposition: Angles -> (90°,90°,90°,90°,90°,90°)
            angles = [m.pi/2]*6

            homing_pose = self.for_kinematic(angles)  # calculate the position via forward kinematics

            self.currPose = homing_pose
            self.currSteps = self.angles2steps(angles)

        else:
            raise Exception('Chosen homing-method is not defined!')

    # KINEMATICS
    def inv_kinematic(self, pose: list):
        """Inverse kinematics of 6-RUS robot:
        
        `pose`: list with numeric content 
        
        `return`: list with all six motor-angles"""

        # convert all inputs to floats to be able to work with complex numbers
        x = float(pose[0])
        y = float(pose[1])
        z = float(pose[2])
        alpha = float(pose[3])
        beta = float(pose[4])
        gamma = float(pose[5])

        # Use given Robot dimensions
        geometic_params = self.geometricParams

        l1 = geometic_params[0]
        l2 = geometic_params[1]
        dx = geometic_params[2]
        dy = geometic_params[3]
        Dx = geometic_params[4]
        Dy = geometic_params[5]

        j = complex(0, 1)  # define complex numer (0 + i)

        # # TODO: manually implement the functions for theta 1-6 for possible time-optimisations
        # # The Caclulations can be found in the MATLAB-example 
        # # Testimplementation for theta_1:
        # sig_4 = z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) + dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))
        # sig_3 = Dx - x - dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma)
        # sig_2 = sig_4**2 + sig_3**2 + (y - Dy + dx*(cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta)) + dy*(cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma)))**2 + l1**2 - l2**2
        # sig_1_squared = (sig_4**2 - (sig_2**2)/(4*l1**2) + sig_3**2)*sig_4**2
    
        # sig_1 = float(sig_1_squared)**0.5

        # yTeil = -(sig_1*sig_3 + (sig_4**2*sig_2)/(2*l1))/((sig_4**2 + sig_3**2)*sig_4)
        # xTeil = (sig_1 - (sig_3*sig_2)/(2*l1))/(sig_4**2 + sig_3**2)

        # theta_1_ = atan2(yTeil.real, xTeil.real)
        
        # calculate motor-angles
        theta_1 = np.angle(((((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) + dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 - ((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) + dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 + (Dx - x - dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2 + (y - Dy + dx*(cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta)) + dy*(cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma)))**2 + l1**2 - l2**2)**2/(4*l1**2) + (Dx - x - dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2)*(z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) + dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2)**(1/2) - ((Dx - x - dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))*((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) + dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 + (Dx - x - dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2 + (y - Dy + dx*(cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta)) + dy*(cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma)))**2 + l1**2 - l2**2))/(2*l1))/((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) + dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 + (Dx - x - dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2) - (((((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) + dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 - ((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) + dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 + (Dx - x - dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2 + (y - Dy + dx*(cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta)) + dy*(cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma)))**2 + l1**2 - l2**2)**2/(4*l1**2) + (Dx - x - dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2)*(z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) + dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2)**(1/2)*(Dx - x - dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma)) + ((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) + dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2*((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) + dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 + (Dx - x - dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2 + (y - Dy + dx*(cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta)) + dy*(cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma)))**2 + l1**2 - l2**2))/(2*l1))*j)/(((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) + dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 + (Dx - x - dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2)*(z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) + dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))))
        theta_2 = np.angle(((((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) - dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 - ((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) - dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 + (Dy + y + dx*(cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta)) - dy*(cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma)))**2 + (x - Dx + dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2 + l1**2 - l2**2)**2/(4*l1**2) + (x - Dx + dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2)*(z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) - dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2)**(1/2) + ((x - Dx + dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))*((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) - dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 + (Dy + y + dx*(cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta)) - dy*(cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma)))**2 + (x - Dx + dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2 + l1**2 - l2**2))/(2*l1))/((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) - dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 + (x - Dx + dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2) + (((((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) - dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 - ((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) - dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 + (Dy + y + dx*(cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta)) - dy*(cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma)))**2 + (x - Dx + dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2 + l1**2 - l2**2)**2/(4*l1**2) + (x - Dx + dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2)*(z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) - dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2)**(1/2)*(x - Dx + dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma)) - ((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) - dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2*((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) - dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 + (Dy + y + dx*(cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta)) - dy*(cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma)))**2 + (x - Dx + dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2 + l1**2 - l2**2))/(2*l1))*j)/(((z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) - dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))**2 + (x - Dx + dx*cos(beta)*cos(gamma) + dy*cos(beta)*sin(gamma))**2)*(z + dx*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta)) - dy*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma)))))
        theta_3 = np.angle((2*((((2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 - (((cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - z + (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 + (Dx/2 + x - (3**(1/2)*Dy)/2 - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2 + (Dy/2 + y + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2))**2 + l1**2 - l2**2)**2/l1**2 + (Dx/2 + x - (3**(1/2)*Dy)/2 + 3**(1/2)*(Dy/2 + y + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2)*(2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2)/4)**(1/2) - ((((cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - z + (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 + (Dx/2 + x - (3**(1/2)*Dy)/2 - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2 + (Dy/2 + y + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2))**2 + l1**2 - l2**2)*(Dx/2 + x - (3**(1/2)*Dy)/2 + 3**(1/2)*(Dy/2 + y + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2)))/l1)/((2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 + (Dx/2 + x - (3**(1/2)*Dy)/2 + 3**(1/2)*(Dy/2 + y + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2) + ((2*((((2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 - (((cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - z + (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 + (Dx/2 + x - (3**(1/2)*Dy)/2 - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2 + (Dy/2 + y + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2))**2 + l1**2 - l2**2)**2/l1**2 + (Dx/2 + x - (3**(1/2)*Dy)/2 + 3**(1/2)*(Dy/2 + y + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2)*(2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2)/4)**(1/2)*(Dx/2 + x - (3**(1/2)*Dy)/2 + 3**(1/2)*(Dy/2 + y + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2)) + ((2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2*(((cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - z + (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 + (Dx/2 + x - (3**(1/2)*Dy)/2 - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2 + (Dy/2 + y + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2))**2 + l1**2 - l2**2))/l1)*j)/(((2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 + (Dx/2 + x - (3**(1/2)*Dy)/2 + 3**(1/2)*(Dy/2 + y + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2)*(2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))))
        theta_4 = np.angle((2*((((2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2 + (Dx/2 + x + (3**(1/2)*Dy)/2 + 3**(1/2)*(y - Dy/2 + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2 - ((Dx/2 + x + (3**(1/2)*Dy)/2 - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2 + (y - Dy/2 + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2))**2 + l1**2 - l2**2 + (z + (cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2)**2/l1**2)*(2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2)/4)**(1/2) - (((Dx/2 + x + (3**(1/2)*Dy)/2 - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2 + (y - Dy/2 + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2))**2 + l1**2 - l2**2 + (z + (cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2)*(Dx/2 + x + (3**(1/2)*Dy)/2 + 3**(1/2)*(y - Dy/2 + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2)))/l1)/((2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2 + (Dx/2 + x + (3**(1/2)*Dy)/2 + 3**(1/2)*(y - Dy/2 + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2) - ((2*((((2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2 + (Dx/2 + x + (3**(1/2)*Dy)/2 + 3**(1/2)*(y - Dy/2 + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2 - ((Dx/2 + x + (3**(1/2)*Dy)/2 - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2 + (y - Dy/2 + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2))**2 + l1**2 - l2**2 + (z + (cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2)**2/l1**2)*(2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2)/4)**(1/2)*(Dx/2 + x + (3**(1/2)*Dy)/2 + 3**(1/2)*(y - Dy/2 + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2)) + ((2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2*((Dx/2 + x + (3**(1/2)*Dy)/2 - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2 + (y - Dy/2 + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2))**2 + l1**2 - l2**2 + (z + (cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2))/l1)*j)/(((2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2 + (Dx/2 + x + (3**(1/2)*Dy)/2 + 3**(1/2)*(y - Dy/2 + (3**(1/2)*Dx)/2 - (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2)*(2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))))
        theta_5 = np.angle((2*((((2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2 - (((cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - z + (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2 + (Dx/2 + x + (3**(1/2)*Dy)/2 - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2 + l1**2 - l2**2 + ((3**(1/2)*Dx)/2 - y - Dy/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2))**2)**2/l1**2 + (Dx/2 + x + (3**(1/2)*Dy)/2 + 3**(1/2)*((3**(1/2)*Dx)/2 - y - Dy/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2)*(2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2)/4)**(1/2) - ((((cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - z + (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2 + (Dx/2 + x + (3**(1/2)*Dy)/2 - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2 + l1**2 - l2**2 + ((3**(1/2)*Dx)/2 - y - Dy/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2))**2)*(Dx/2 + x + (3**(1/2)*Dy)/2 + 3**(1/2)*((3**(1/2)*Dx)/2 - y - Dy/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2)))/l1)/((2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2 + (Dx/2 + x + (3**(1/2)*Dy)/2 + 3**(1/2)*((3**(1/2)*Dx)/2 - y - Dy/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2) + ((2*((((2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2 - (((cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - z + (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2 + (Dx/2 + x + (3**(1/2)*Dy)/2 - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2 + l1**2 - l2**2 + ((3**(1/2)*Dx)/2 - y - Dy/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2))**2)**2/l1**2 + (Dx/2 + x + (3**(1/2)*Dy)/2 + 3**(1/2)*((3**(1/2)*Dx)/2 - y - Dy/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2)*(2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2)/4)**(1/2)*(Dx/2 + x + (3**(1/2)*Dy)/2 + 3**(1/2)*((3**(1/2)*Dx)/2 - y - Dy/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2)) + ((2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2*(((cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - z + (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2 + (Dx/2 + x + (3**(1/2)*Dy)/2 - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2 + l1**2 - l2**2 + ((3**(1/2)*Dx)/2 - y - Dy/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2))**2))/l1)*j)/(((2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))**2 + (Dx/2 + x + (3**(1/2)*Dy)/2 + 3**(1/2)*((3**(1/2)*Dx)/2 - y - Dy/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2) + (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 + (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 - (3**(1/2)*dx)/2))**2)*(2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 - (3**(1/2)*dx)/2) - 2*z + 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 + (3**(1/2)*dy)/2))))
        theta_6 = np.angle((2*((((2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 - (((3**(1/2)*Dy)/2 - x - Dx/2 + cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2 + l1**2 - l2**2 + (z + (cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 + (Dy/2 - y + (3**(1/2)*Dx)/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2))**2)**2/l1**2 + (Dx/2 + x - (3**(1/2)*Dy)/2 + 3**(1/2)*(Dy/2 - y + (3**(1/2)*Dx)/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2)*(2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2)/4)**(1/2) - ((((3**(1/2)*Dy)/2 - x - Dx/2 + cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2 + l1**2 - l2**2 + (z + (cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 + (Dy/2 - y + (3**(1/2)*Dx)/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2))**2)*(Dx/2 + x - (3**(1/2)*Dy)/2 + 3**(1/2)*(Dy/2 - y + (3**(1/2)*Dx)/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2)))/l1)/((2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 + (Dx/2 + x - (3**(1/2)*Dy)/2 + 3**(1/2)*(Dy/2 - y + (3**(1/2)*Dx)/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2) - ((2*((((2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 - (((3**(1/2)*Dy)/2 - x - Dx/2 + cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2 + l1**2 - l2**2 + (z + (cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 + (Dy/2 - y + (3**(1/2)*Dx)/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2))**2)**2/l1**2 + (Dx/2 + x - (3**(1/2)*Dy)/2 + 3**(1/2)*(Dy/2 - y + (3**(1/2)*Dx)/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2)*(2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2)/4)**(1/2)*(Dx/2 + x - (3**(1/2)*Dy)/2 + 3**(1/2)*(Dy/2 - y + (3**(1/2)*Dx)/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2)) + ((2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2*(((3**(1/2)*Dy)/2 - x - Dx/2 + cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) + cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2 + l1**2 - l2**2 + (z + (cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - (sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 + (Dy/2 - y + (3**(1/2)*Dx)/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2))**2))/l1)*j)/(((2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))**2 + (Dx/2 + x - (3**(1/2)*Dy)/2 + 3**(1/2)*(Dy/2 - y + (3**(1/2)*Dx)/2 + (cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2) - (cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2)) - cos(beta)*cos(gamma)*(dx/2 - (3**(1/2)*dy)/2) - cos(beta)*sin(gamma)*(dy/2 + (3**(1/2)*dx)/2))**2)*(2*z + 2*(cos(gamma)*sin(alpha) + cos(alpha)*sin(beta)*sin(gamma))*(dy/2 + (3**(1/2)*dx)/2) - 2*(sin(alpha)*sin(gamma) - cos(alpha)*cos(gamma)*sin(beta))*(dx/2 - (3**(1/2)*dy)/2))))

        return [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]

    def for_kinematic(self, angles):
        """Forward kinematics of 6-RUS robot. This is done with a numeric solve (fsolve)

        `angles`: list of angles in the form of [θ1, θ2, θ3, θ4, θ5, θ6]

        `return`: list with pose in the form of [x, y, z, α, β, γ]"""
        
        angles_as_np_array = np.array(angles)  # convert to numpy array to subtract from another array
        
        # create function to minimize
        def func(x, h1=angles_as_np_array):
            """This function returns the difference between the current position (`H1`) and a guess (`X`).
            It is used for the numeric fsolve."""

            motor_angles = self.inv_kinematic(x)
            h2 = np.array(motor_angles)

            difference = h1 - h2  # calculate the difference between calulated and real angles

            return difference
        
        # initial guess/startingvalue
        x_0 = [0.0, 0.0, -130.0, 0.0, 0.0, 0.0]

        curr_pose = fsolve(func, x_0)  # solve numerically with initial guess

        return list(curr_pose)

    def change_robot_dimensions(self, l1, l2, dx, dy, Dx, Dy):
        """This changes the dimensions of the robot which are important for the kinematics.
        This has to be done before homing or moving of the robot.
        See the documentation for the kinematics to see which values belong to which robot dimension"""
        self.geometricParams = [l1, l2, dx, dy, Dx, Dy]


if __name__ == '__main__':
    # Example Code if this file gets executed directly
    robo = sixRUS(stepper_mode=1 / 32, step_delay=0.001)  # initialise robot
    robo.homing('90')  # homing of robot with method ‘90’

    print('Homingpose: ', robo.currPose)

    # define two poses
    pose1 = [0, 0, -90, 0, 0, 0]
    pose2 = [0, 0, -127, 0, 0, 0]

    robo.mov(pose1)  # move to first pose (PTP) 
    time.sleep(.5)  # wait 0.5 seconds

    robo.mov(pose2)  # move to second pose (PTP) 
    time.sleep(.5)

    robo.mov(pose1)  # move to first pose again 

    GPIO.cleanup()  # cleanup GPIO-Pins
