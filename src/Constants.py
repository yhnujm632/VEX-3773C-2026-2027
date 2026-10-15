from VEXLib.Geometry.GeometryUtil import circle_circumference
from VEXLib.Geometry.Rotation2d import Rotation2d
from VEXLib.Geometry.Translation1d import Distance
from VEXLib.Geometry.Velocity1d import Velocity1d
from VEXLib.Sensors.Controller import ControlStyles
from vex import Ports, GearSetting, Brain, Colordesc

NO_LOGGING = True

class PIDGains:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd


class PIDFGains(PIDGains):
    def __init__(self, kp, ki, kd, kf=0.0):
        super().__init__(kp, ki, kd)
        self.kf = kf


class DefaultPreferences:
    CONTROL_STYLE = ControlStyles.SPLIT_ARCADE
    CUBIC_FILTER_LINEARITY = 1.0
    MOVE_SPEED = 1.0
    TURN_SPEED = 1.0
    DO_TURN_DECAY = False
    USE_PIDF_CONTROL = False
    PIDF_GAINS_LEFT_DRIVER = PIDFGains(0.05, 0, 0, 0.6)
    PIDF_GAINS_RIGHT_DRIVER = PIDFGains(0.05, 0, 0, 0.6)

    PIDF_GAINS_LEFT_AUTO = PIDFGains(0.45, 0.1, 0, 0.7)
    PIDF_GAINS_RIGHT_AUTO = PIDFGains(0.45, 0.1, 0, 0.7)
    ENABLE_DRIVING = True
    INPUT_DEBUG_MODE = False


class RossPreferences(DefaultPreferences):
    CONTROL_STYLE = ControlStyles.SPLIT_ARCADE
    CUBIC_FILTER_LINEARITY = 1
    USE_PIDF_CONTROL = True

class DebugPreferences(DefaultPreferences):
    CONTROL_STYLE = ControlStyles.SPLIT_ARCADE
    CUBIC_FILTER_LINEARITY = 1
    USE_PIDF_CONTROL = True
    INPUT_DEBUG_MODE = True
    ENABLE_DRIVING = False


class CompetitionSmartPorts:
    """Drivetrain"""

    FRONT_LEFT_DRIVETRAIN_MOTOR = Ports.PORT1
    REAR_LOWER_LEFT_DRIVETRAIN_MOTOR = Ports.PORT11
    REAR_UPPER_LEFT_DRIVETRAIN_MOTOR = Ports.PORT12

    FRONT_RIGHT_DRIVETRAIN_MOTOR = Ports.PORT10
    REAR_LOWER_RIGHT_DRIVETRAIN_MOTOR = Ports.PORT20
    REAR_UPPER_RIGHT_DRIVETRAIN_MOTOR = Ports.PORT19

    FLOATING_INTAKE_MOTOR = Ports.PORT2
    LEVER_MOTOR = Ports.PORT3

    INERTIAL_SENSOR = Ports.PORT21
    # LEFT_DISTANCE = Ports.PORT0
    # RIGHT_DISTANCE = Ports.PORT0
    # VISION_SENSOR = Ports.PORT0

    OPTICAL_SENSOR = Ports.PORT4

class ThreeWirePorts:
    brain = Brain()
    RAISE_SOLENOID = brain.three_wire_port.g
    PISTON_ACTUATOR_SOLENOID = brain.three_wire_port.b
    DESCORING_ARM_SOLENOID_UP = brain.three_wire_port.h
    DESCORING_ARM_SOLENOID_OUT = brain.three_wire_port.a
    HOOD_SOLENOID = brain.three_wire_port.c


class GearRatios:
    DRIVETRAIN = GearSetting.RATIO_6_1
    LEVER_MOTOR = GearSetting.RATIO_36_1
    FLOATING_INTAKE = GearSetting.RATIO_6_1

class IntakeConstants:
    RETURN_SPEED = 25
    SCORE_POSITION = 120


class DrivetrainProperties:
    POSITION_PID_GAINS = PIDGains(6, 0.6, 0.0)
    ROTATION_PID_GAINS = PIDGains(0.5, 0.0, 0.03)

    ROBOT_RELATIVE_TO_FIELD_RELATIVE_ROTATION = Rotation2d.from_degrees(90)
    STARTUP_ANGLE = Rotation2d.from_degrees(90)
    TURN_TIMEOUT_SECONDS = 1.5
    TURN_CORRECTION_SCALAR_WHILE_MOVING = 0.5
    TURNING_THRESHOLD = Rotation2d.from_degrees(1)
    MOVEMENT_DISTANCE_THRESHOLD = Distance.from_centimeters(1)
    MOVEMENT_MAX_EXTRA_TIME = 1.6
    MAX_ACHIEVABLE_SPEED = Velocity1d.from_meters_per_second(2.205) ##TODO test
    MOTOR_TO_WHEEL_GEAR_RATIO = 36 / 64
    WHEEL_DIAMETER = Distance.from_inches(3.233)
    WHEEL_CIRCUMFERENCE = circle_circumference(WHEEL_DIAMETER / 2)
    TRACK_WIDTH = Distance.from_inches(11.4)

    LONG_GOAL_COLOR_DESC = Colordesc(1, 129, 66, 35, 12, 0.2)

