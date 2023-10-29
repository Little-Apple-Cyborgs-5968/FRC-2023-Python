import rev
from sim.sparksim import CANSparkMax
from wpilib import SPI
from wpilib.drive import MecanumDrive
from navx import AHRS

from robot_map import CAN

class DriveTrain:
    def __init__(self, controller):
        self.BALANCE = False

        # Intializes motors for the drive basse.
        self.frontRightMotor = CANSparkMax(CAN.frontRightChannel, rev.CANSparkMax.MotorType.kBrushless)
        self.rearRightMotor = CANSparkMax(CAN.rearRightChannel, rev.CANSparkMax.MotorType.kBrushless)
        self.frontLeftMotor = CANSparkMax(CAN.frontLeftChannel, rev.CANSparkMax.MotorType.kBrushless)
        self.rearLeftMotor = CANSparkMax(CAN.rearLeftChannel, rev.CANSparkMax.MotorType.kBrushless)
        self.frontRightMotor.restoreFactoryDefaults()
        self.rearRightMotor.restoreFactoryDefaults()
        self.frontLeftMotor.restoreFactoryDefaults()
        self.rearLeftMotor.restoreFactoryDefaults()
        self.frontRightMotor.setInverted(True)
        self.rearRightMotor.setInverted(True)

        # Sets up the controller and drive train.
        self.controller = controller
        self.robotDrive = MecanumDrive(self.frontLeftMotor, self.rearLeftMotor, self.frontRightMotor,
                                       self.rearRightMotor)
        self.gyroscope = AHRS(SPI.Port.kMXP)
        self.gyroscope.reset()
        
    def autoBalance(self):
        roll = self.gyroscope.getRoll() - self.intialRoll
        if (abs(roll) > 1):
            self.robotDrive.driveCartesian(roll/100, 0, 0)
        else:
            self.robotDrive.driveCartesian(0, 0, 0)

        # print(f"Roll: {roll}")

    def autonomousInit(self):
        self.intialRoll = self.gyroscope.getRoll()
        self.robotDrive.setSafetyEnabled(False)
    
    def autonomousPeriodic(self):
        if (self.BALANCE):
            self.autoBalance()

    def teleopInit(self):
        self.intialRoll = self.gyroscope.getRoll()
        self.robotDrive.setSafetyEnabled(True)

    def teleopPeriodic(self):
        # Handles the movement of the drive base.
        self.robotDrive.driveCartesian(
            -self.controller.getLeftY(),
            self.controller.getLeftX(),
            0.5 * self.controller.getRightX(),
            -self.gyroscope.getRotation2d(),
        )

        if self.controller.getLeftBumper():
            self.autoBalance()