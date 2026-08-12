

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

    def run_intake_and_raise_descorer(self, speed):
        self.robot.intake.run_intake(speed)
        self.robot.descoring_arm.extend()

    def run_upper_intake_and_raise_descorer(self, speed):
        self.robot.intake.run_upper_intake(speed)
        self.robot.descoring_arm.extend()

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

        return Rotation2d.from_degrees(180)

    def execute(self):
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(20), 180)



class Right4Long(AutonomousRoutine):
    name = "Right 4 Long" #add hook at the end
    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(-90)

    def execute(self):
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(31.5), -90, max_extra_time=0)
        self.robot.match_load_helper.extend()
        self.robot.intake.run_floating_intake(1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(10), 180)

        # Pick up #1 from match loader
        time.sleep(0.1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-32), 180, max_extra_time=0, turn_first=False)
        self.robot.match_load_helper.retract()


        self.robot.intake.extend_flap()
        self.robot.intake.set_lever_velocity(100)
        time.sleep(0.5)
        self.robot.intake.retract_flap()
        self.robot.intake.move_lever_to_position(0)
        self.robot.descoring_arm.wing_out_and_down()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(15), 180, turn_first=False)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-15), 125)
        self.set_speed_factor(0.4)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-28), 180)
        self.robot.drivetrain.set_speed(Velocity1d.from_zero(), Velocity1d.from_zero())
        while True:
            self.robot.drivetrain.update_powers()


class Left4Long(AutonomousRoutine):
    name = "Left 4 Long" #Add Hook
    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(90)

    def execute(self):
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(31.5), 90, max_extra_time=0)
        self.robot.match_load_helper.extend()
        self.robot.intake.run_floating_intake(1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(10), 180)

        # Pick up #1 from match loader
        time.sleep(0.1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-32), 180, max_extra_time=0, turn_first=False)
        self.robot.match_load_helper.retract()


        self.robot.intake.extend_flap()
        self.robot.intake.set_lever_velocity(100)
        time.sleep(0.5)
        self.robot.intake.retract_flap()
        self.robot.intake.move_lever_to_position(0)
        self.robot.descoring_arm.wing_out_and_down()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(15), 180, turn_first=False)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-15), 125)
        self.set_speed_factor(0.4)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-28), 180)
        while True:
            self.robot.drivetrain.update_powers()

class Right6Long(AutonomousRoutine):
    name = "Right 6 Long"
    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(-12.5)

    def execute(self):
        self.robot.intake.run_floating_intake(1)
        self.set_acceleration_factor(0.6)
        self.set_speed_factor(0.6)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(23), -12.5)
        # self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(5), -12.5)
        self.set_acceleration_factor(1)

        self.robot.match_load_helper.extend()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(28), -120)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(21), -180, max_extra_time=0)
        # Picking up
        self.robot.drivetrain.set_powers(0.1, 0.1)
        self.robot.intake.intake_until_color(bad_color(self.robot), timeout=3)
        self.robot.drivetrain.set_powers(0, 0)
        # self.robot.drivetrain.back_up_to_goal(-0.5)
        # TODO: JANKY ANGLE
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-4), -185, turn_first=False)
        self.robot.intake.retract_flap()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-30), -185, max_extra_time=0)

        self.robot.intake.extend_flap()
        self.robot.intake.set_lever_velocity(30)
        self.robot.intake.run_floating_intake(1)
        start_time = time.time()
        while not is_near(self.robot.intake.get_lever_position(), IntakeConstants.SCORE_POSITION, 5) and (time.time() - start_time < 2):
            time.sleep_ms(5)
        self.robot.intake.stop_floating_intake()
        self.robot.descoring_arm.wing_out_and_down()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(14), 180, turn_first=False)
        self.robot.match_load_helper.retract()
        Thread(lambda: self.robot.intake.move_lever_to_position(0))
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-17), 125)
        self.set_speed_factor(0.4)
        self.robot.intake.run_floating_intake(-1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-23), 182)
        while True:
            self.robot.drivetrain.update_powers()

    def cleanup(self):
        self.robot.intake.retract_flap()
        self.robot.intake.stop_floating_intake()

class Left6Long(AutonomousRoutine):
    name = "Left 6 Long"
    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(12.5)

    def execute(self):
        self.robot.intake.run_floating_intake(1)
        self.set_acceleration_factor(0.6)
        self.set_speed_factor(0.6)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(23), 12.5)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(5), 12.5)
        self.set_acceleration_factor(1)

        self.robot.match_load_helper.extend()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(27), 120)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(21), 180, max_extra_time=0)
        # Picking up
        self.robot.drivetrain.set_powers(0.1, 0.1)
        self.robot.intake.intake_until_color(bad_color(self.robot), timeout=3)
        self.robot.drivetrain.set_powers(0, 0)
        # self.robot.drivetrain.back_up_to_goal(-0.5)
        # TODO: JANKY ANGLE ON OTHER AUTO
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-4), 180, turn_first=False)
        self.robot.intake.retract_flap()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-30), 180, max_extra_time=0)

        self.robot.intake.extend_flap()
        self.robot.intake.set_lever_velocity(30)
        self.robot.intake.run_floating_intake(1)
        start_time = time.time()
        while not is_near(self.robot.intake.get_lever_position(), IntakeConstants.SCORE_POSITION, 5) and (time.time() - start_time < 2):
            time.sleep_ms(5)
        self.robot.intake.stop_floating_intake()
        self.robot.descoring_arm.wing_out_and_down()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(14), 180, turn_first=False)
        self.robot.match_load_helper.retract()
        Thread(lambda: self.robot.intake.move_lever_to_position(0))
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-17), 125)
        self.set_speed_factor(0.4)
        self.robot.intake.run_floating_intake(-1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-23), 182)
        while True:
            self.robot.drivetrain.update_powers()

    def cleanup(self):
        self.robot.intake.retract_flap()
        self.robot.intake.stop_floating_intake()


class GoalLineTest(AutonomousRoutine):
    name = "Goal Line Test"
    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(180)

    def pre_match_setup(self):
        pass

    def execute(self):
        self.robot.drivetrain.back_up_to_goal(-0.1)

class Left2Mid5Long(AutonomousRoutine):
    name = "Left 2 Mid 5 Long"
    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(9)

    def pre_match_setup(self):
        pass

    def execute(self):
        self.robot.intake.run_floating_intake(1)
        self.set_acceleration_factor(0.8)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(37), 9, commands=[TimeBasedCommand(-0.8, lambda: self.robot.match_load_helper.extend())])
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-7), 9)
        self.robot.intake.lower_intake()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-12.5), 134)
        self.robot.intake.extend_flap()
        self.robot.intake.move_lever_to_position(110)
        self.robot.intake.retract_flap()
        self.robot.intake.move_lever_to_position(0)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(46), 134, turn_first=False)
        self.robot.intake.raise_intake()
        self.robot.match_load_helper.extend()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(15), 180)
        # Pickup from match loader
        self.robot.drivetrain.set_powers(0.1, 0.1)
        self.robot.intake.intake_until_color(bad_color(self.robot), timeout=3)
        self.robot.match_load_helper.retract()
        self.robot.drivetrain.set_powers(0, 0)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-5), 180, turn_first=False)
        self.robot.intake.run_floating_intake(1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-29), 180)
        # Score in Long Goal
        self.robot.intake.extend_flap()
        self.robot.intake.set_lever_velocity(80)
        start_time = time.time()
        while not is_near(self.robot.intake.get_lever_position(), IntakeConstants.SCORE_POSITION, 5) and (time.time() - start_time < 2):
            time.sleep_ms(5)
        self.robot.intake.retract_flap()
        self.robot.descoring_arm.wing_out_and_down()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(15), 180)
        Thread(lambda: self.robot.intake.move_lever_to_position(0))
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-16), -225)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-17), 180)
        while True:
            self.robot.drivetrain.set_speed(Velocity1d.from_meters_per_second(0), Velocity1d.from_meters_per_second(0))
            self.robot.drivetrain.update_powers()


class Right4Long3Mid(AutonomousRoutine):
    name = "Right 3 Mid 4 Long"

    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(-90)

    def execute(self):
        self.robot.match_load_helper.extend()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(34), -90)
        self.robot.intake.run_floating_intake(1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(16), 180)

        # Pickup from match loader
        self.robot.drivetrain.set_powers(0.1, 0.1)
        self.robot.intake.intake_until_color(bad_color(self.robot), timeout=1)
        self.robot.drivetrain.set_powers(0, 0)

        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-34), 180)

        # Score
        self.robot.intake.run_floating_intake(1)
        self.robot.intake.extend_flap()
        self.robot.intake.set_lever_velocity(100)
        self.robot.drivetrain.set_powers(-0.2, -0.2)

        start_time = time.time()

        while not is_near(self.robot.intake.get_lever_position(), IntakeConstants.SCORE_POSITION, 5) and (time.time() - start_time < 2):
            time.sleep_ms(5)
            if self.robot.intake.get_lever_position() > 45:
                self.robot.intake.run_floating_intake(-1)
            else:
                self.robot.intake.run_floating_intake(1)


        self.robot.match_load_helper.retract()
        self.robot.intake.run_floating_intake(-1)
        self.robot.intake.retract_flap()
        Thread(lambda: self.robot.intake.move_lever_to_position(0))
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(0), 60)
        self.set_speed_factor(0.2)
        self.robot.intake.run_floating_intake(1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(19), 60)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(26), 47)
        self.robot.intake.run_floating_intake(-0.6)
        time.sleep_ms(50)
        self.set_speed_factor(1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-30), 45)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-25), 90)
        self.robot.drivetrain.turn_to(Rotation2d.from_degrees(180))
        self.robot.descoring_arm.wing_out_and_down()
        self.set_speed_factor(0.5)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-25), 180)


class Skills(AutonomousRoutine):
    name = "Skills"

    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(90)

    def execute(self):
        pass


class WorldsWinPoint(AutonomousRoutine):
    name = "Worlds Win Point"

    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(-90)

    def execute(self):
        self.robot.match_load_helper.extend()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(32), -90)
        self.robot.intake.run_floating_intake(1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(16), 180)

        # Pickup from match loader
        self.robot.drivetrain.set_powers(0.1, 0.1)
        self.robot.intake.intake_until_color(bad_color(self.robot), timeout=3)
        self.robot.drivetrain.set_powers(0, 0)


        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-5), 180, turn_first=False)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-29), 180)

        # Score
        self.robot.intake.run_floating_intake(1)
        self.robot.intake.extend_flap()
        self.robot.intake.set_lever_velocity(80)
        self.robot.drivetrain.set_powers(-0.2, -0.2)

        start_time = time.time()

        while not is_near(self.robot.intake.get_lever_position(), IntakeConstants.SCORE_POSITION, 5) and (time.time() - start_time < 2):
            time.sleep_ms(5)
        self.robot.intake.retract_flap()
        self.robot.intake.run_floating_intake(-1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(17), 180)
        Thread(lambda: self.robot.intake.move_lever_to_position(0))
        self.robot.match_load_helper.retract()
        self.robot.intake.run_floating_intake(1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(24), 45)
        self.robot.intake.run_floating_intake(1)
        self.set_speed_factor(0.3)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(30), 45)
        self.robot.intake.run_floating_intake(-0.6)
        time.sleep_ms(10)
        self.set_speed_factor(1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-14), 45)
        self.robot.intake.run_floating_intake(1)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(33), 90)
        self.set_speed_factor(0.3)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(8), 90)
        self.set_speed_factor(1)
        self.robot.intake.lower_intake()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-25), 135)
        # Score
        self.robot.intake.run_floating_intake(1)
        self.robot.intake.extend_flap()
        self.robot.intake.set_lever_velocity(80)
        self.robot.drivetrain.set_powers(-0.2, -0.2)

        start_time = time.time()

        while not is_near(self.robot.intake.get_lever_position(), IntakeConstants.SCORE_POSITION, 5) and (time.time() - start_time < 2):
            time.sleep_ms(5)
        self.robot.intake.retract_flap()



class SketchyWorldsWinPoint(AutonomousRoutine):
    name = "Sketchy Worlds Win Point"

    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(-90)

    def execute(self):
        self.set_acceleration_factor(1.2)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(21), -90, max_extra_time=0)
        self.set_acceleration_factor(1)
        self.robot.match_load_helper.extend()
        # time.sleep(0.5)
        self.robot.drivetrain.arc_movement(arc_angle=Rotation2d.from_degrees(90),
                                           arc_radius=Translation1d.from_inches(9),
                                           direction="CW",
                                           start_direction_degrees=-90,
                                           turn_first=False)
        self.robot.intake.pickup()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(9), 180, turn_first=False)

        # Pick up #1 from match loader
        self.robot.drivetrain.set_powers(0.1, 0.1)
        # time.sleep(0.1)
        self.set_acceleration_factor(1.2)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-32), 180, max_extra_time=0, turn_first=False, commands=[TimeBasedCommand(0.1, self.robot.intake.stop_upper_intake), TimeBasedCommand(-0.5, lambda: self.robot.intake.run_hood(1)), TimeBasedCommand(-0.5, lambda: self.robot.intake.run_upper_intake(1))])
        self.robot.match_load_helper.retract()
        # self.robot.drivetrain.set_powers(-0.1, -0.1)

        self.robot.intake.run_hood(1)
        time.sleep(1)

        self.robot.intake.run_hood(0)
        self.set_acceleration_factor(0.8)
        # self.robot.drivetrain.arc_movement(arc_angle=Rotation2d.from_degrees(155),
        #                                    arc_radius=Translation1d.from_inches(9),
        #                                    direction="CW",
        #                                    start_direction_degrees=-185,
        #                                    turn_first=False)
        self.robot.drivetrain.turn_to(Rotation2d.from_degrees(70))
        # self.robot.drivetrain.arc_movement(arc_angle=Rotation2d.from_degrees(60),
        #                                        arc_radius=Translation1d.from_inches(14),
        #                                    direction="CCW",
        #                                    start_direction_degrees=30,
        #                                    turn_first=False,
        #                                    commands=[TimeBasedCommand(-0.2, self.robot.match_load_helper.extend)])

        self.set_acceleration_factor(1.3)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(15), 70, max_extra_time=0, turn_first=False)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(50), 90, commands=[TimeBasedCommand(0, self.robot.match_load_helper.retract), TimeBasedCommand(-0.5, self.robot.match_load_helper.extend)])
        self.set_acceleration_factor(1)
        self.robot.intake.lower_intake()
        self.robot.midgoal_hood_actuator.extend()
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-6), 90)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-11), 135)
        self.robot.intake.run_hood(1)
        time.sleep(1)
        self.robot.intake.run_hood(0)
        self.robot.intake.raise_intake()
        self.set_acceleration_factor(1.2)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(45), 135, turn_first=False)
        self.robot.midgoal_hood_actuator.retract()
        self.set_acceleration_factor(2)
        self.robot.drivetrain.move_distance_towards_direction_trap(Translation1d.from_inches(-20), 180, commands=[TimeBasedCommand(0, self.robot.match_load_helper.retract), TimeBasedCommand(-0.4, lambda: self.robot.intake.run_hood(1))])
        self.robot.intake.run_hood(1)


class ColorTest(AutonomousRoutine):
    name = "Color Test"
    def __init__(self, robot: Robot):
        super().__init__(robot)

    @staticmethod
    def startup_angle():
        return Rotation2d.from_degrees(180)

    def execute(self):
        self.robot.intake.run_intake(1)
        self.robot.intake.intake_until_color(bad_color(self.robot))

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




all_routines = [WorldsWinPoint, Left6Long, Right6Long, Left4Long, Right4Long, Left2Mid5Long, Right4Long3Mid, Drive, GoalLineTest]
