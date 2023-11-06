import wpilib
from wpilib import simulation
from wpimath.system.plant import DCMotor
from wpimath.controller import PIDController

from logger import Logger
from robot import MyRobot

class ArmSim:
    LENGTH = 1 # meter (starting length)
    WEIGHT = 5 # kg
    FACING = -1 # -1 for facing left, 1 facing right
    OFFSET = 15 # degrees the arm starts off of straight down
    DOWN_ANGLE = -90 + FACING * OFFSET
    UP_ANGLE = DOWN_ANGLE + FACING * 180
    MIN_ANGLE = min(DOWN_ANGLE, UP_ANGLE)
    MAX_ANGLE = max(DOWN_ANGLE, UP_ANGLE)
    DEGREES_PER_POS = 4.4

    def __init__(self, robot: "MyRobot"):
        self.log = Logger('Arm')
        # moment of inertia, estimated by arm length and weight.
        self.MOI = simulation.SingleJointedArmSim.estimateMOI(
            self.LENGTH,
            self.WEIGHT,
        )
        self.sim = simulation.SingleJointedArmSim(
            DCMotor.NEO(1), # arm gearbox with one NEO motor
            600.0, # Gear ratio (reduction)
            self.MOI,
            self.LENGTH,
            self.MIN_ANGLE,
            self.MAX_ANGLE,
            True, # yes, simulate gravity
        )
        # no idea how to specify a starting angle for this sim, so let's just rotate
        # it till we get where we want (i.e., the DOWN_ANGLE)
        pid = PIDController(1, 0, 0)
        pid.setSetpoint(self.DOWN_ANGLE)
        pid.setTolerance(0.5)
        while not pid.atSetpoint():
            angle = self.sim.getAngle()
            self.sim.setInput(0, pid.calculate(angle))
            self.sim.update(2)
        starting_angle = self.sim.getAngle()

        ### Now draw our arm on the SmartDashboard
        self.mech2d = wpilib.Mechanism2d(60, 60)
        # put shoulder in the center
        self.mech_shoulder = self.mech2d.getRoot("Shoulder", 30, 30)
        # draw a tower down to the bottom
        self.mech_tower = self.mech_shoulder.appendLigament(
            "Tower", 30, -90, 6, wpilib.Color8Bit(wpilib.Color.kBlue)
        )
        # and add an arm, also starting at the shoulder
        self.mech_arm = self.mech_shoulder.appendLigament(
            "Arm", 30, starting_angle, 6, wpilib.Color8Bit(wpilib.Color.kYellow)
        )
        wpilib.SmartDashboard.putData("Arm Sim", self.mech2d)

        self.shoulderEncoder = robot.Arm.shoulderEncoder
        self.shoulderMotor = robot.Arm.shoulderMotor
        self.shoulderPID = robot.Arm.shoulderPIDController

        print('shoulderPID:'
              f' P={self.shoulderPID.getP()}'
              f' I={self.shoulderPID.getI()}'
              f' D={self.shoulderPID.getD()}'
        )
        print(f'angles: min={self.MIN_ANGLE} max={self.MAX_ANGLE}'
              f' starting={starting_angle}'
        )

    def update(self, now: float, tm_diff: float) -> None:
        ### Update shoulder
        pos_offset = self.getPosOffset()
        # for now, just add 0.5 in place of FF
        pid_calc = self.FACING * (self.shoulderPID.calculate(pos_offset) + 0.5)

        self.sim.setInputVoltage(1.5 * pid_calc)
        self.sim.update(1)

        # set simulated encoder's readings
        angle = self.sim.getAngle()
        pos_offset = self.getPosOffset(angle)
        self.shoulderEncoder.setPosition(pos_offset)
        # Update the mechanism arm angle based on the simulated arm angle
        self.mech_arm.setAngle(angle)

        self.log.stagger(now, f'angle={angle:.1f} pos_offset={pos_offset:.1f}'
            f' shoulder set point={self.shoulderPID.getSetpoint():.1f}'
            f' shoulder pid_calc={pid_calc:.6f}'
        )
    
    # Encoder started at 0 pos, and each pos is DEGREES_PER_POS,
    # so convert our current angle offset to a pos offset.
    def getPosOffset(self, angle: float = None) -> float:
        if angle == None:
            angle = self.sim.getAngle()
        return self.FACING * (angle - self.DOWN_ANGLE) / self.DEGREES_PER_POS