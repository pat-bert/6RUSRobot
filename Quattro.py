from Robot import Robot


class Quattro(Robot):
    def __init__(self, stepper_mode=1 / 32, steps_per_rev=200, step_delay=0.0208):
        super().__init__(dof=4, stepper_mode=stepper_mode, steps_per_rev=steps_per_rev, step_delay=step_delay)

    def inv_kinematic(self, pose: list):
        pass

    def forward_kinematic(self, angles):
        pass

    def change_robot_dimensions(self, *args):
        pass
