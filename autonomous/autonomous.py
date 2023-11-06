from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state

class Autonomous(StatefulAutonomous):
    MODE_NAME = "Auto Mode"
    DEFAULT = True

    def initialize(self): 
        self.initial_called = None

    @timed_state(duration=0.5, next_state="extender1", first=True)
    def intake1(self):
        self.Arm.intakeMotor.set(1)

    @timed_state(duration=2, next_state="shoulder1")
    def extender1(self):
        self.Arm.intakeMotor.set(0)
        self.Arm.extenderPosition = -10

    @timed_state(duration=2, next_state="drive1")
    def shoulder1(self):
        self.Arm.shoulderPosition = 18

    @timed_state(duration=0.35, next_state="extender2")
    def drive1(self):
        self.DriveTrain.robotDrive.driveCartesian(-0.2,0,0)

    @timed_state(duration=2,  next_state="intake2")
    def extender2(self):
        self.DriveTrain.robotDrive.driveCartesian(0,0,0)
        self.Arm.extenderPosition = 20

    @timed_state(duration=0.5, next_state="extender3")
    def intake2(self):
        self.Arm.intakeMotor.set(-1)

    @timed_state(duration=2, next_state="shoulder2")
    def extender3(self):
        self.Arm.intakeMotor.set(0)
        self.Arm.extenderPosition = -10
    
    @timed_state(duration=2, next_state="drive2")
    def shoulder2(self):
        self.Arm.shoulderPosition = 5

    @timed_state(duration=1.6, next_state="balance")
    def drive2(self):
        # mobility
        self.DriveTrain.robotDrive.driveCartesian(0.6, 0, 0)

    @timed_state(duration=5)
    def balance(self):
        self.DriveTrain.robotDrive.driveCartesian(0, 0, 0)
        # self.DriveTrain.BALANCE = True