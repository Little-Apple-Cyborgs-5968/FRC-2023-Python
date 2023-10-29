import wpilib
import wpilib.drive
import rev
from components.drive_train import DriveTrain
from components.arm import Arm
from robot_map import USB
from robotpy_ext.autonomous import AutonomousModeSelector


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """This function is called upon program startup."""
        self.controller = wpilib.XboxController(USB.controllerChannel)
        self.DriveTrain = DriveTrain(self.controller)
        self.Arm = Arm(self.controller)
        self.components = {"DriveTrain": self.DriveTrain,
                           "Arm": self.Arm}
        self.auto = AutonomousModeSelector("autonomous", self.components)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.Arm.autonomousInit()
        self.DriveTrain.autonomousInit()
        self.auto.start()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.Arm.autonomousPeriodic()
        self.DriveTrain.autonomousPeriodic()
        self.auto.periodic()
    
    def disabledInit(self):
        self.auto.disable()

    def teleopInit(self):
        """This function is run once each time the robot enters teleoperated mode."""
        self.DriveTrain.teleopInit()
        self.Arm.teleopInit()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.DriveTrain.teleopPeriodic()
        self.Arm.teleopPeriodic()
        if self.controller.getStartButton():
            print('hit start button!!!!')
            self.Arm.shoulderEncoder.setPosition(0)
            self.Arm.extenderEncoder.setPosition(0)
        if self.controller.getBackButton(): # keyboard M
            # resets the yaw
            self.DriveTrain.gyroscope.reset()


if __name__ == "__main__":
    wpilib.run(MyRobot)
