from wpilib import simulation
from wpimath.geometry import Pose2d

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import drivetrains
from pyfrc.physics.units import units

from logger import Logger
from robot import MyRobot
from robot_map import CAN

class DriveSim:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller
        self.log = Logger('Drive')

        # sim drive motors
        self.fl_motor = simulation.PWMSim(CAN.frontLeftChannel)
        self.rl_motor = simulation.PWMSim(CAN.rearLeftChannel)
        self.fr_motor = simulation.PWMSim(CAN.frontRightChannel)
        self.rr_motor = simulation.PWMSim(CAN.rearRightChannel)

        # Gyro
        self.navx = simulation.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx.getDouble("Yaw")

        self.drivetrain = drivetrains.MecanumDrivetrain(
            x_wheelbase = 30 * units.inches,
            y_wheelbase = 30 * units.inches,
            speed       =  5 * units.fps
        )

        # place robot in front of a blue grid station
        initialPose = Pose2d.fromFeet(7.5, 9.75, 0 * units.degrees)
        self.physics_controller.field.setRobotPose(initialPose)

    def update(self, now: float, tm_diff: float) -> None:
        fl_speed = self.fl_motor.getSpeed()
        rl_speed = self.rl_motor.getSpeed()
        fr_speed = self.fr_motor.getSpeed()
        rr_speed = self.rr_motor.getSpeed()

        speeds = self.drivetrain.calculate(fl_speed, rl_speed, fr_speed, rr_speed)
        pose = self.physics_controller.drive(speeds, tm_diff)
        pose_deg = pose.rotation().degrees()

        # Update the gyro simulation
        # NavX yaw is positive clockwise, but the field is positive counter-clockwise
        yaw = -pose_deg
        self.navx_yaw.set(yaw)

        self.log.stagger(now, f'{speeds} pose_deg={pose_deg:.1f} yaw={yaw:.1f}')