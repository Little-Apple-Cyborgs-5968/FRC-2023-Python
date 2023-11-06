from pyfrc.physics.core import PhysicsInterface

from sim.drive_sim import DriveSim
from sim.arm_sim import ArmSim
from robot import MyRobot

class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        """
        self.drive_sim = DriveSim(physics_controller, robot)
        self.arm_sim = ArmSim(robot)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        self.drive_sim.update(now, tm_diff)
        self.arm_sim.update(now, tm_diff)