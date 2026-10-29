

from VEXLib.Math import is_near
from VEXLib.Util.Logging import Logger

from VEXLib.Geometry.Rotation2d import Rotation2d
from VEXLib.Geometry.Translation1d import Translation1d
from VEXLib.Geometry.Velocity1d import Velocity1d
from TankDrivetrain import TimeBasedCommand
import VEXLib.Util.time as time
from Constants import IntakeConstants
from vex import Color, Thread, DEGREES

if False:
    from CompetitionRobot import Robot

autonomous_log = Logger("logs/Autonomous")

def bad_color(robot):
    if robot.alliance_color == Color.RED:
        return Color.BLUE
    else:
        return Color.RED

class AutonomousRoutine:
    name = "AutonomousRoutine"
    def __init__(self, robot: Robot):
        self.robot = robot

    def __repr__(self):
        return self.name

    def __str__(self):
        return self.name

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(0)

    def pre_match_setup(self):
        pass

    def execute(self):
        return False

    def set_acceleration_factor(self, factor):
        self.robot.drivetrain.acceleration_scalar = factor

    def set_speed_factor(self, factor):
        self.robot.drivetrain.velocity_scalar = factor

    def shake(self, iterations):
        self.robot.drivetrain.set_powers(0.3, 0.3)
        time.sleep(0.2)
        for _ in range(iterations):
            self.robot.drivetrain.set_powers(0.3, 0.3)
            time.sleep(0.2)
            self.robot.drivetrain.set_powers(-0.3, -0.3)
            time.sleep(0.1)
        self.robot.drivetrain.set_powers(0, 0)

    def cleanup(self):
        self.robot.drivetrain.init()
        self.robot.drivetrain.set_powers(0, 0)
        self.robot.flush_all_logs()


class Drive(AutonomousRoutine):
    name = "Drive"
    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():

        return Rotation2d.from_degrees(0)

    def execute(self):
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(20), 0)


class LeftAllianceGoal(AutonomousRoutine):
    name = "Left Alliance Goal"
    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(0)

    def execute(self):
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_centimeters(58), 0, commands=[self.robot.intake.go_to_step(2), -0.2])
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_centimeters(70), 90)
        self.robot.intake.score()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_centimeters(-10), 90)
        self.robot.intake.lower_elevator()


class RightAllianceGoal(AutonomousRoutine):
    name = "Right Alliance Goal"
    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(0)

    def execute(self):
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_centimeters(58), 0, commands=[self.robot.intake.go_to_step(2), -0.2])
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_centimeters(70), -90)
        self.robot.intake.score()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_centimeters(-10), -90)
        self.robot.intake.lower_elevator()


class Square(AutonomousRoutine):
    name = "Square"
    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(180)

    def execute(self):
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(24), 180)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(24), -90)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(24), 0)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(24), 90)




all_routines = [LeftAllianceGoal, RightAllianceGoal, Drive]
