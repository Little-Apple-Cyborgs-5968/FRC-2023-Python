import rev
import wpilib
from wpilib import simulation
from wpimath.controller import PIDController

if wpilib.RobotBase.isSimulation():

    class SparkMaxRelativeEncoder(wpilib.AnalogEncoder):
        def __init__(self, channel) -> None:
            super().__init__(channel)
            self._velocity = 0
            self._pos = 0

        def getVelocity(self):
            return self._velocity
        def setVelocity(self, vel: float):
            self._velocity = vel
        
        def getPosition(self):
            return self._pos
        def setPosition(self, pos: float):
            self._pos = pos
    
    class SparkMaxPidController(PIDController):
        def __init__(self) -> None:
            super().__init__(0, 0, 0)
        def setFF(self, ff):
            pass
        def setReference(self, value: float, ctrl: rev.CANSparkMaxLowLevel.ControlType): 
            if (ctrl != rev.CANSparkMax.ControlType.kPosition):
                raise(f'control type {ctrl} not implemented')
            self.setSetpoint(value)

    class CANSparkMax(wpilib.Spark):
        def __init__(self, channel: int, ignored) -> None:
            super().__init__(channel)
            self._encoder = SparkMaxRelativeEncoder(channel)
            self._pidController = SparkMaxPidController()

        def getEncoder(self):
            return self._encoder
        
        def getPIDController(self):
            return self._pidController

        def restoreFactoryDefaults(self):
            pass
        def setIdleMode(self, mode):
            pass

else:
    import rev

    CANSparkMax = rev.CANSparkMax