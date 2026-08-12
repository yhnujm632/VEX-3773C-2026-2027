import json
import math

import VEXLib
import VEXLib.Math.MathUtil as MathUtil
from Constants import DrivetrainProperties, NO_LOGGING, DefaultPreferences, CompetitionSmartPorts
from VEXLib.Util.Logging import NoLogger
from VEXLib.Algorithms.LinearRegressor import LinearRegressor
from VEXLib.Algorithms.PID import PIDController
from VEXLib.Algorithms.PIDF import PIDFController
from VEXLib.Algorithms.RateOfChangeCalculator import RateOfChangeCalculator
from VEXLib.Algorithms.TrapezoidProfile import *
from VEXLib.Geometry import GeometryUtil
from VEXLib.Geometry.Pose2d import Pose2d
from VEXLib.Geometry.Rotation2d import Rotation2d
from VEXLib.Geometry.Translation1d import Translation1d
from VEXLib.Geometry.Translation2d import Translation2d
from VEXLib.Geometry.Velocity1d import Velocity1d
from VEXLib.Kinematics.TankOdometry import TankOdometry
from VEXLib.Math import clamp, average, is_near
from VEXLib.Motor import Motor
from VEXLib.Units import Units
from VEXLib.Util import time
from VEXLib.Util.Logging import Logger, TimeSeriesLogger
from VEXLib.Util.motor_analysis import collect_power_relationship_data
from vex import DEGREES, Thread, Distance, DistanceUnits

SmartPorts = CompetitionSmartPorts

if NO_LOGGING:
    drivetrain_log = NoLogger("logs/Drivetrain")
    debug_log = NoLogger("logs/DrivetrainDebug")
else:
    drivetrain_log = Logger("logs/Drivetrain")
    debug_log = Logger("logs/DrivetrainDebug")


class TimeBasedCommand:
    def __init__(self, time, function, background=False):
        self.time = time
        self.function = function
        self.has_been_executed = False
        self.background = background

    def execute_once(self):
        if not self.has_been_executed:
            if self.background:
                Thread(self.function)
            else:
                self.function()
            self.has_been_executed = True


class Drivetrain:
    def __init__(
            self,
            left_motors: list[Motor],
            right_motors: list[Motor],
            inertial_sensor,
            vision,
    ):
        self.log = drivetrain_log
        self.debug_log = debug_log
        self.log.trace("Initializing Drivetrain class")

        self.left_motors = left_motors
        self.right_motors = right_motors

        self.left_distance = Distance(SmartPorts.LEFT_DISTANCE)
        self.right_distance = Distance(SmartPorts.RIGHT_DISTANCE)

        self.vision = vision

        self.odometry = TankOdometry(
            inertial_sensor,
            Rotation2d.from_degrees(180),
        )
        self.log.debug("Odometry initialized with inertial_sensor:", inertial_sensor)

        self.ANGLE_DIRECTION = 1
        self.log.debug("ANGLE_DIRECTION set to", self.ANGLE_DIRECTION)

        self.velocity_scalar = 1
        self.acceleration_scalar = 1

        self.left_drivetrain_PID = PIDFController(
            DefaultPreferences.PIDF_GAINS_LEFT_DRIVER, t=1e-5
        )
        self.right_drivetrain_PID = PIDFController(
            DefaultPreferences.PIDF_GAINS_RIGHT_DRIVER, t=1e-5
        )

        self.left_speed = 0
        self.right_speed = 0

        self.left_drivetrain_speed_calculator = RateOfChangeCalculator(
            minimum_sample_time=0.075
        )
        self.right_drivetrain_speed_calculator = RateOfChangeCalculator(
            minimum_sample_time=0.075
        )

        self.left_position_pid = PIDController(DrivetrainProperties.POSITION_PID_GAINS, 1e-5, 10)
        self.right_position_pid = PIDController(DrivetrainProperties.POSITION_PID_GAINS, 1e-5, 10)

        self.rotation_PID = PIDController(
            DrivetrainProperties.ROTATION_PID_GAINS, Units.milliseconds_to_seconds(10), 10
        )
        self.log.debug("Position and Rotation PID Controllers initialized with gains")

        self.left_trapezoidal_profile = TrapezoidProfile(
            Constraints(
                DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second(),
                DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second(),
            )
        )
        self.right_trapezoidal_profile = TrapezoidProfile(
            Constraints(
                DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second(),
                DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second(),
            )
        )

        self.log.debug("Trapezoidal profiles initialized")

        self.TURNING_THRESHOLD = DrivetrainProperties.TURNING_THRESHOLD
        self.log.debug("Turning threshold set to ", self.TURNING_THRESHOLD.to_degrees())

        self.target_pose = Pose2d(Translation2d(), self.odometry.zero_rotation)
        self.log.debug("Target pose initialized to", self.target_pose)

    def set_angles_inverted(self, inverted):
        self.log.debug("Set angles to inverted: ", inverted)
        self.ANGLE_DIRECTION = -1 if inverted else 1

    def update_odometry(self):
        self.odometry.update(self.get_left_distance(), self.get_right_distance())

    def update_target_translation(self, distance: Translation1d, rotation: Rotation2d):
        self.log.trace("Entering update_target_position")
        self.target_pose.rotation = rotation
        rotation += DrivetrainProperties.ROBOT_RELATIVE_TO_FIELD_RELATIVE_ROTATION
        self.target_pose.translation += Translation2d(
            distance * rotation.cos(), distance * rotation.sin()
        )
        self.log.debug("New target translation: {} in".format(self.target_pose.translation.to_inches()))

    def get_distance_and_angle_from_position(self, target_translation: Translation2d):
        self.log.trace("Entering get_distance_and_angle_from_position")
        delta_translation = target_translation - self.target_pose.translation
        return (
            delta_translation.length(),
            delta_translation.angle()
            - DrivetrainProperties.ROBOT_RELATIVE_TO_FIELD_RELATIVE_ROTATION,
        )

    def set_powers(self, left_power, right_power):
        for motor in self.left_motors:
            motor.set(left_power)
        for motor in self.right_motors:
            motor.set(right_power)

    def update_powers(self):
        self.update_drivetrain_velocities()
        current_left_speed, current_right_speed = self.get_speeds()
        left_controller_output = self.left_drivetrain_PID.update(
            current_left_speed.to_meters_per_second()
        )
        right_controller_output = self.right_drivetrain_PID.update(
            current_right_speed.to_meters_per_second()
        )

        self.set_powers(left_controller_output, right_controller_output)

    def update_drivetrain_velocities(self):
        #TODO: Clean up this logic
        if self.left_drivetrain_speed_calculator.ready_for_sample(
                time.time()
        ):
            self.left_speed = self.left_drivetrain_speed_calculator.calculate_rate(
                self.get_left_distance().to_meters(), time.time()
            )
        if self.right_drivetrain_speed_calculator.ready_for_sample(
                time.time()
        ):
            self.right_speed = self.right_drivetrain_speed_calculator.calculate_rate(
                self.get_right_distance().to_meters(), time.time()
            )

    def get_left_speed(self):
        return Velocity1d.from_meters_per_second(self.left_speed)

    def get_right_speed(self):
        return Velocity1d.from_meters_per_second(self.right_speed)

    def get_speeds(self):
        return self.get_left_speed(), self.get_right_speed()

    def get_left_distance(self) -> Translation1d:
        motor_rotation_degrees = MathUtil.average_iterable(
            [motor.position(DEGREES) for motor in self.left_motors]
        )

        wheel_rotation = Rotation2d.from_degrees(
            motor_rotation_degrees * DrivetrainProperties.MOTOR_TO_WHEEL_GEAR_RATIO
        )
        return GeometryUtil.arc_length_from_rotation(
            DrivetrainProperties.WHEEL_CIRCUMFERENCE, wheel_rotation
        )

    def get_right_distance(self) -> Translation1d:
        motor_rotation_degrees = MathUtil.average_iterable(
            [motor.position(DEGREES) for motor in self.right_motors]
        )

        wheel_rotation = Rotation2d.from_degrees(
            motor_rotation_degrees * DrivetrainProperties.MOTOR_TO_WHEEL_GEAR_RATIO
        )
        return GeometryUtil.arc_length_from_rotation(
            DrivetrainProperties.WHEEL_CIRCUMFERENCE, wheel_rotation
        )

    def get_distance_from_object(self):
        return Translation1d.from_millimeters(MathUtil.average(self.left_distance.object_distance(units=DistanceUnits.MM), self.right_distance.object_distance(units=DistanceUnits.MM)))

    def set_speed_zero_to_one(self, left_speed, right_speed):
        self.set_speed(
            DrivetrainProperties.MAX_ACHIEVABLE_SPEED * left_speed,
            DrivetrainProperties.MAX_ACHIEVABLE_SPEED * right_speed,
            )

    def set_speed(self, left_speed: Velocity1d, right_speed: Velocity1d):
        self.left_drivetrain_PID.setpoint = left_speed.to_meters_per_second()
        self.right_drivetrain_PID.setpoint = right_speed.to_meters_per_second()

    def turn_to(self, rotation: Rotation2d):
        self.log.trace("Entering turn_to")
        self.log.info("Turning to", rotation.to_degrees(), "degrees")

        old_target_heading = self.rotation_PID.setpoint

        new_target_heading = rotation.to_radians() * self.ANGLE_DIRECTION

        self.log.debug(
            "old_target_heading",
            Units.radians_to_degrees(old_target_heading),
            "degrees",
        )
        self.log.debug(
            "new_target_heading",
            Units.radians_to_degrees(new_target_heading),
            "degrees",
        )

        angular_difference = MathUtil.smallest_angular_difference(
            old_target_heading, new_target_heading
        )
        self.log.debug(
            "Optimized angular_difference",
            Units.radians_to_degrees(angular_difference),
            "degrees",
        )
        self.log.debug("New target heading", new_target_heading, "degrees")

        self.rotation_PID.setpoint = self.rotation_PID.setpoint + angular_difference
        self.update_odometry()

        self.left_drivetrain_PID.reset(self.get_left_distance().to_meters())
        self.right_drivetrain_PID.reset(self.get_right_distance().to_meters())
        self.rotation_PID.reset(self.odometry.get_rotation().to_radians())
        start_time = time.time()
        while True:
            pid_output = -self.rotation_PID.update(self.odometry.get_rotation().to_radians())
            debug_log.log_vars({
                "turning_pid_output": pid_output,
                "current_heading (rev)": self.odometry.get_rotation().to_revolutions(),
                "target_heading (rev)": Rotation2d.from_radians(self.rotation_PID.setpoint).to_revolutions(),
            })

            if pid_output > 0.0:
                pid_output += 0.03
            else:
                pid_output -= 0.03

            # output = MathUtil.clamp(
            #     pid_output,
            #     -0.6,
            #     0.6,
            # )
            output = pid_output

            self.set_speed_zero_to_one(output, -output)
            self.update_powers()
            self.update_odometry()
            # is_turning_slowly_enough = abs(self.odometry.inertial_sensor.gyro_rate(ZAXIS)) < DrivetrainProperties.TURNING_VELOCITY_THRESHOLD.to_degrees()
            is_at_setpoint = self.rotation_PID.at_setpoint(self.odometry.get_rotation().to_radians(), threshold=self.TURNING_THRESHOLD.to_radians())
            wheels_are_moving = abs(self.get_right_speed().to_centimeters_per_second()) + abs(self.get_right_speed().to_centimeters_per_second()) > 5
            time_exceeded = (time.time() - start_time) > DrivetrainProperties.TURN_TIMEOUT_SECONDS
            if (is_at_setpoint and not wheels_are_moving) or time_exceeded:
                drivetrain_log.info("Turn finished, at_setpoint: {}, wheels_are_moving: {}, time_exceeded: {}".format(is_at_setpoint, wheels_are_moving, time_exceeded))

                break
        if not self.rotation_PID.at_setpoint(
                self.odometry.get_rotation().to_radians(),
                threshold=self.TURNING_THRESHOLD.to_radians()
        ):
            self.log.warn(
                "Turn timed out, still",
                Units.radians_to_degrees(
                    self.rotation_PID.setpoint
                    - self.odometry.get_rotation().to_radians()
                ),
                "degrees away",
            )
        self.set_speed_zero_to_one(0, 0)
        self.set_powers(0, 0)
        self.left_drivetrain_PID.reset(self.get_left_distance().to_meters())
        self.right_drivetrain_PID.reset(self.get_right_distance().to_meters())
        self.rotation_PID.reset(self.odometry.get_rotation().to_radians())

    def move_to_point(self, translation: Translation2d, use_back=False, turn=True, stop_immediately=False, commands=None):
        self.log.trace("Entering move_to_point")
        distance, angle = self.get_distance_and_angle_from_position(translation)
        if use_back:
            angle += Rotation2d.from_revolutions(0.5)
            distance = distance.inverse()
        self.move_distance_towards_direction_trap(distance, (angle.to_degrees() if turn else Units.radians_to_degrees(self.rotation_PID.setpoint)), stop_immediately=stop_immediately, commands=commands)

    def arc_movement(
            self,
            arc_angle,
            arc_radius,
            direction,
            start_direction_degrees,
            turn_first=True,
            turn_correct=True,
            stop_immediately=False,
            commands=None,
            max_extra_time=DrivetrainProperties.MOVEMENT_MAX_EXTRA_TIME,
            dont_stop=False,
    ):
        # self.log.trace("Entering move_distance_towards_direction_trap")
        # self.log.debug(
        #     "Driving",
        #     ("forwards" if distance.to_meters() > 0 else "backwards"),
        #     distance.to_inches(),
        #     "in at",
        #     direction_degrees,
        #     "degrees",
        # )

        if commands is None:
            commands = []

        self.left_drivetrain_PID.reset(self.get_left_distance().to_meters())
        self.right_drivetrain_PID.reset(self.get_right_distance().to_meters())
        self.rotation_PID.reset(self.odometry.get_rotation().to_radians())

        if turn_first:
            self.turn_to(Rotation2d.from_degrees(start_direction_degrees))

        self.left_drivetrain_PID.reset(self.get_left_distance().to_meters())
        self.right_drivetrain_PID.reset(self.get_right_distance().to_meters())
        self.rotation_PID.reset(self.odometry.get_rotation().to_radians())

        left_start_position = self.get_left_distance().to_meters()
        right_start_position = self.get_right_distance().to_meters()

        full_circle_arc_length = arc_radius.to_meters() * math.pi * 2

        arc_length = full_circle_arc_length * arc_angle.to_revolutions()

        inner_ratio = 1 - (DrivetrainProperties.TRACK_WIDTH.to_meters() / arc_radius.to_meters()) / 2
        outer_ratio = 1 + (DrivetrainProperties.TRACK_WIDTH.to_meters() / arc_radius.to_meters()) / 2

        if direction == "CCW":
            left_ratio = inner_ratio
            right_ratio = outer_ratio
        else:
            left_ratio = outer_ratio
            right_ratio = inner_ratio

        left_arc_length = arc_length * left_ratio
        right_arc_length = arc_length * right_ratio

        left_initial_state = State(left_start_position, self.get_left_speed().to_meters_per_second())
        right_initial_state = State(right_start_position, self.get_right_speed().to_meters_per_second())

        self.log.debug("Initial states")
        self.log.log_vars({
            "left_start_position": left_start_position,
            "right_start_position": right_start_position,
            "current_left_velocity": self.get_left_speed().to_meters_per_second(),
            "current_right_speed": self.get_right_speed().to_meters_per_second(),
        })

        if dont_stop:
            left_goal_state = State(left_start_position + left_arc_length, DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second() * 0.5 * left_ratio)
            right_goal_state = State(right_start_position + right_arc_length, DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second() * 0.5 * right_ratio)
        else:
            left_goal_state = State(left_start_position + left_arc_length, 0)
            right_goal_state = State(right_start_position + right_arc_length, 0)

        start_time = time.time()
        self.left_trapezoidal_profile = TrapezoidProfile(
            Constraints(
                DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second() * 0.5 * left_ratio,
                DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second() * left_ratio * self.acceleration_scalar,
                )
        )

        self.right_trapezoidal_profile = TrapezoidProfile(
            Constraints(
                DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second() * 0.5 * right_ratio,
                DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second() * right_ratio * self.acceleration_scalar,
                )
        )

        self.left_trapezoidal_profile.calculate(0, left_initial_state, left_goal_state)
        self.right_trapezoidal_profile.calculate(0, right_initial_state, right_goal_state)
        total_time = max(self.left_trapezoidal_profile.total_time(), self.right_trapezoidal_profile.total_time())

        while True:
            elapsed_time = time.time() - start_time
            target_left_state = self.left_trapezoidal_profile.calculate(
                elapsed_time, left_initial_state, left_goal_state
            )
            target_right_state = self.right_trapezoidal_profile.calculate(
                elapsed_time, right_initial_state, right_goal_state
            )

            remaining_time = total_time - elapsed_time

            for command in commands:
                if command.time >= 0:
                    # Reference from start of movement
                    if command.time <= elapsed_time:
                        command.execute_once()

                else:
                    # Reference from end of movement
                    if (-command.time) >= remaining_time:
                        command.execute_once()

            left_position = self.get_left_distance().to_meters()
            right_position = self.get_right_distance().to_meters()

            self.left_position_pid.setpoint = target_left_state.position
            self.right_position_pid.setpoint = target_right_state.position

            # TODO: Implement rotation correction
            rotation_output = 0

            self.set_speed_zero_to_one(
                self.left_position_pid.update(left_position) + rotation_output, self.right_position_pid.update(right_position) - rotation_output
            )
            self.update_powers()
            self.update_odometry()

            is_near_end = is_near(left_position, left_goal_state.position, DrivetrainProperties.MOVEMENT_DISTANCE_THRESHOLD.to_meters()) and is_near(right_position, right_goal_state.position, DrivetrainProperties.MOVEMENT_DISTANCE_THRESHOLD.to_meters())


            if stop_immediately:
                time_exceeded = (
                        elapsed_time >= total_time
                )
            else:
                time_exceeded = (
                        elapsed_time >= total_time + max_extra_time
                )
            if elapsed_time >= total_time or (is_near_end or time_exceeded):
                self.log.debug(
                    "Terminating movement: is_near_end: ",
                    is_near_end,
                    "time_exceeded: ",
                    time_exceeded,
                )
                if time_exceeded:
                    self.log.warn("time_exceeded")
                break

        if not dont_stop:
            self.set_speed_zero_to_one(0, 0)
            self.set_powers(0, 0)
            self.left_drivetrain_PID.reset(self.get_left_distance().to_meters())
            self.right_drivetrain_PID.reset(self.get_right_distance().to_meters())
            self.rotation_PID.reset(self.odometry.get_rotation().to_radians())

        self.update_odometry()
        self.rotation_PID.setpoint = self.rotation_PID.setpoint + MathUtil.smallest_angular_difference(self.rotation_PID.setpoint, Units.degrees_to_radians(start_direction_degrees + arc_angle.to_radians()))



    def move_distance_towards_direction_trap(
            self,
            distance: Translation1d,
            direction_degrees,
            turn_first=True,
            turn_correct=True,
            stop_immediately=False,
            commands=None,
            max_extra_time=DrivetrainProperties.MOVEMENT_MAX_EXTRA_TIME,
            dont_stop=False,
            end_velocity=None,
    ):
        self.log.trace("Entering move_distance_towards_direction_trap")
        self.log.debug(
            "Driving",
            ("forwards" if distance.to_meters() > 0 else "backwards"),
            distance.to_inches(),
            "in at",
            direction_degrees,
            "degrees",
        )

        if commands is None:
            commands = []

        self.left_drivetrain_PID.reset(self.get_left_distance().to_meters())
        self.right_drivetrain_PID.reset(self.get_right_distance().to_meters())
        self.rotation_PID.reset(self.odometry.get_rotation().to_radians())

        if turn_first:
            self.turn_to(Rotation2d.from_degrees(direction_degrees))
        else:
            self.rotation_PID.setpoint = self.rotation_PID.setpoint + MathUtil.smallest_angular_difference(self.rotation_PID.setpoint, Units.degrees_to_radians(direction_degrees))

        self.left_drivetrain_PID.reset(self.get_left_distance().to_meters())
        self.right_drivetrain_PID.reset(self.get_right_distance().to_meters())
        self.rotation_PID.reset(self.odometry.get_rotation().to_radians())

        left_start_position = self.get_left_distance().to_meters()
        right_start_position = self.get_right_distance().to_meters()

        self.left_trapezoidal_profile = TrapezoidProfile(
            Constraints(
                DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second() * self.velocity_scalar,
                DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second() * self.acceleration_scalar,
                )
        )

        self.right_trapezoidal_profile = TrapezoidProfile(
            Constraints(
                DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second() * self.velocity_scalar,
                DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second() * self.acceleration_scalar,
                )
        )

        left_initial_state = State(left_start_position, MathUtil.average(self.get_left_speed().to_meters_per_second(), self.get_right_speed().to_meters_per_second()))
        right_initial_state = State(right_start_position, MathUtil.average(self.get_left_speed().to_meters_per_second(), self.get_right_speed().to_meters_per_second()))

        if dont_stop:
            left_goal_state = State(left_start_position + distance.to_meters(), DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second() * self.velocity_scalar)
            right_goal_state = State(right_start_position + distance.to_meters(), DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second() * self.velocity_scalar)
        else:
            left_goal_state = State(left_start_position + distance.to_meters(), 0)
            right_goal_state = State(right_start_position + distance.to_meters(), 0)

        if end_velocity is not None:
            left_goal_state = State(left_start_position + distance.to_meters(), DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second() * end_velocity)
            right_goal_state = State(right_start_position + distance.to_meters(), DrivetrainProperties.MAX_ACHIEVABLE_SPEED.to_meters_per_second() * end_velocity)


        start_time = time.time()

        self.left_trapezoidal_profile.calculate(0, left_initial_state, left_goal_state)
        self.right_trapezoidal_profile.calculate(0, right_initial_state, right_goal_state)

        total_time = max(self.left_trapezoidal_profile.total_time(), self.right_trapezoidal_profile.total_time())

        while True:
            elapsed_time = time.time() - start_time
            left_target_state = self.left_trapezoidal_profile.calculate(
                elapsed_time, left_initial_state, left_goal_state
            )
            right_target_state = self.right_trapezoidal_profile.calculate(
                elapsed_time, right_initial_state, right_goal_state
            )

            remaining_time = total_time - elapsed_time

            for command in commands:
                if command.time >= 0:
                    # Reference from start of movement
                    if command.time <= elapsed_time:
                        command.execute_once()

                else:
                    # Reference from end of movement
                    if (-command.time) >= remaining_time:
                        command.execute_once()

            left_position = self.get_left_distance().to_meters()
            right_position = self.get_right_distance().to_meters()

            self.left_position_pid.setpoint = left_target_state.position
            self.right_position_pid.setpoint = right_target_state.position

            left_output_speed = self.left_position_pid.update(left_position)
            right_output_speed = self.right_position_pid.update(right_position)

            if turn_correct:
                rotation_output = (
                        -self.rotation_PID.update(self.odometry.get_rotation().to_radians())
                        * DrivetrainProperties.TURN_CORRECTION_SCALAR_WHILE_MOVING
                )
            else:
                rotation_output = 0

            self.set_speed_zero_to_one(
                left_output_speed, right_output_speed - (rotation_output * 2)
            )
            self.update_powers()
            self.update_odometry()

            # at_setpoint = self.left_position_pid.at_setpoint(
            #     left_position,
            #     DrivetrainProperties.MOVEMENT_DISTANCE_THRESHOLD.to_meters()
            # ) and self.right_position_pid.at_setpoint(
            #     right_position,
            #     DrivetrainProperties.MOVEMENT_DISTANCE_THRESHOLD.to_meters()
            # )

            is_near_end = is_near(left_position, left_goal_state.position, DrivetrainProperties.MOVEMENT_DISTANCE_THRESHOLD.to_meters()) and is_near(right_position, right_goal_state.position, DrivetrainProperties.MOVEMENT_DISTANCE_THRESHOLD.to_meters())

            if stop_immediately:
                time_exceeded = (
                        elapsed_time >= total_time
                )
            else:
                time_exceeded = (
                        elapsed_time >= total_time + max_extra_time
                )
            if elapsed_time >= total_time or (is_near_end or time_exceeded):
                # self.log.debug(
                #     "Terminating movement: is_near_end: ",
                #     is_near_end,
                #     "time_exceeded: ",
                #     time_exceeded,
                # )
                if time_exceeded:
                    self.log.warn("time_exceeded")
                break
            # debug_log.log_vars({
            #     "left_speed_pid_output": left_output_speed,
            #     "right_speed_pid_output": right_output_speed,
            #     "current_left_position": left_position,
            #     "current_right_position": right_position,
            #     "current_left_velocity": self.get_left_speed().to_meters_per_second(),
            #     "current_right_speed": self.get_right_speed().to_meters_per_second(),
            #     "left_target_position": self.left_position_pid.setpoint,
            #     "right_target_position": self.right_position_pid.setpoint,
            #     "current_heading (rev)": self.odometry.get_rotation().to_revolutions(),
            #     "target_heading (rev)": Rotation2d.from_radians(self.rotation_PID.setpoint).to_revolutions(),
            # })
        if not dont_stop:
            self.set_speed_zero_to_one(0, 0)
            self.set_powers(0, 0)
            self.left_drivetrain_PID.reset(self.get_left_distance().to_meters())
            self.right_drivetrain_PID.reset(self.get_right_distance().to_meters())
            self.rotation_PID.reset(self.odometry.get_rotation().to_radians())

        # self.log.debug(
        #     "Remaining Distance: " + str(distance.to_inches() - Units.meters_to_inches(average(left_position, right_position))) + " in"
        # )
        # self.log.debug("Distance Traveled: " + str(Units.meters_to_inches(average(left_position, right_position))) + " in")

        # self.update_target_translation(
        #     distance, Rotation2d.from_degrees(direction_degrees)
        # )

    def move_until_distance_away(self, distance, direction_degrees, turn_correct=True, turn_first=True):

        if turn_first:
            self.turn_to(Rotation2d.from_degrees(direction_degrees))

        while True:
            error = self.get_distance_from_object() - distance
            forward_speed = clamp(error, -0.8, 0.8)

            if turn_correct:
                rotation_output = (
                        -self.rotation_PID.update(self.odometry.get_rotation().to_radians())
                        * DrivetrainProperties.TURN_CORRECTION_SCALAR_WHILE_MOVING
                )
            else:
                rotation_output = 0

            self.set_speed_zero_to_one(forward_speed+rotation_output, forward_speed-rotation_output)

            if self.get_distance_from_object() <= distance:
                break

    def back_up_to_goal(self, speed):
        start_time = time.time()
        while True:
            amount = self.vision.get_adjustment_amount()
            if abs(amount) < 0.1:
                left = -amount
                right = amount
            else:
                left = speed - amount
                right = speed + amount

            self.set_speed_zero_to_one(left, right)
            self.update_powers()
            if time.time() - start_time < 3:
                break


    def update_zero_pose(self, new_zero_pose):
        self.log.trace("Entering update_zero_pose")
        self.odometry.zero_pose = new_zero_pose

    def init(self):
        self.log.debug("Entering init")
        self.velocity_scalar = 1
        self.acceleration_scalar = 1
        self.left_drivetrain_PID.reset(self.get_left_distance().to_meters())
        self.right_drivetrain_PID.reset(self.get_right_distance().to_meters())
        self.rotation_PID.reset(self.odometry.get_rotation().to_radians())

    def set_pose(self):
        self.log.trace("Entering set_pose")
        self.odometry.pose = Pose2d()
        self.odometry.inertial_sensor.reset_rotation()

    def measure_properties(self, each_motor=False):
        self.log.trace("Entering measure_properties")
        self.log.info("Measuring drivetrain properties...")
        if each_motor:
            for i, motor in VEXLib.Util.enumerate(self.left_motors):
                collect_power_relationship_data(
                    "logs/left_{}_drivetrain.csv".format(i), [self.left_motors[i]]
                )
            for i, motor in VEXLib.Util.enumerate(self.right_motors):
                collect_power_relationship_data(
                    "logs/right_{}_drivetrain.csv".format(i), [self.right_motors[i]]
                )
        else:
            collect_power_relationship_data(
                "logs/left_drivetrain.csv", self.left_motors
            )
            self.log.info("Measured left drivetrain properties...")
            collect_power_relationship_data(
                "logs/right_drivetrain.csv", self.right_motors
            )
        self.log.info("Measured right drivetrain properties...")

    def debug(self, imperial=False):
        # self.log.trace("Entering debug")
        data = {"time (s)": time.time()}
        data["rotation (deg)"] = self.odometry.pose.rotation.to_degrees()
        if imperial:
            data["left_position (in)"] = self.get_left_distance().to_inches()
            data["right_position (in)"] = self.get_right_distance().to_inches()
            data["left_speed (in/s)"] = self.get_left_speed().to_inches_per_second()
            data["right_speed (in/s)"] = self.get_right_speed().to_inches_per_second()
            data["x position (in)"] = (
                self.odometry.pose.translation.x_component.to_inches()
            )
            data["y position (in)"] = (
                self.odometry.pose.translation.y_component.to_inches()
            )
        else:
            data["left_position (cm)"] = self.get_left_distance().to_centimeters()
            data["right_position (cm)"] = self.get_right_distance().to_centimeters()
            data["left_speed (cm/s)"] = (
                self.get_left_speed().to_centimeters_per_second()
            )
            data["right_speed (cm/s)"] = (
                self.get_right_speed().to_centimeters_per_second()
            )
            data["x position (cm)"] = (
                self.odometry.pose.translation.x_component.to_centimeters()
            )
            data["y position (cm)"] = (
                self.odometry.pose.translation.y_component.to_centimeters()
            )
            self.log.log_vars(data)
        return data

    def verify_speed_pid(self):
        self.log.trace("Entering verify_speed_pid")
        self.left_drivetrain_PID.pid_gains = DefaultPreferences.PIDF_GAINS_LEFT_AUTO
        self.right_drivetrain_PID.pid_gains = DefaultPreferences.PIDF_GAINS_RIGHT_AUTO
        self.log.info("Initializing time series logger")
        speed_logger = TimeSeriesLogger("logs/speed_pid_data.csv", self.debug().keys())

        self.log.info("Start verifying speed PID")
        self.set_speed(
            Velocity1d.from_centimeters_per_second(50),
            Velocity1d.from_centimeters_per_second(50),
        )
        self.log.debug("Setting speeds to +/+50 cm/sec for 3 seconds")
        start_time = time.time()
        while time.time() - start_time < 3:
            self.update_odometry()
            self.update_powers()
            speed_logger.write_data(self.debug())
        self.log.debug("Setting speeds to -/-50 cm/sec for 3 seconds")
        self.set_speed(
            Velocity1d.from_centimeters_per_second(-50),
            Velocity1d.from_centimeters_per_second(-50),
        )
        start_time = time.time()
        while time.time() - start_time < 3:
            self.update_odometry()
            self.update_powers()
            data = self.debug()
            speed_logger.write_data(data)
        self.log.debug("Setting speeds to +/-50 cm/sec for 3 seconds")
        self.set_speed(
            Velocity1d.from_centimeters_per_second(50),
            Velocity1d.from_centimeters_per_second(-50),
        )
        start_time = time.time()
        while time.time() - start_time < 3:
            self.update_odometry()
            self.update_powers()
            data = self.debug()
            speed_logger.write_data(data)
        self.log.debug("Setting speeds to +/+160 cm/sec for 3 seconds")
        self.set_speed(
            Velocity1d.from_centimeters_per_second(160),
            Velocity1d.from_centimeters_per_second(160),
        )
        start_time = time.time()
        while time.time() - start_time < 3:
            self.update_odometry()
            self.update_powers()
            data = self.debug()
            speed_logger.write_data(data)
        self.set_speed(
            Velocity1d.from_centimeters_per_second(0),
            Velocity1d.from_centimeters_per_second(0),
        )
        self.update_odometry()
        self.set_powers(0, 0)

    def determine_speed_pid_constants(self):
        self.log.trace("Entering determine_speed_pid_constants")
        # Collect power relationship data
        data_json_left = collect_power_relationship_data(
            "left_data.csv",
            self.left_motors,
            speed_function=lambda motor: (
                (self.update_drivetrain_velocities(), self.get_speeds())[1][0]
            ).to_meters_per_second(),
        )
        data_json_right = collect_power_relationship_data(
            "right_data.csv",
            self.right_motors,
            speed_function=lambda motor: (
                (self.update_drivetrain_velocities(), self.get_speeds())[1][1]
            ).to_meters_per_second(),
        )
        self.log.info(data_json_left)
        self.log.info(data_json_right)
        data_left = json.loads(data_json_left)
        data_right = json.loads(data_json_right)

        # Perform linear regression to find kF
        regressor_left = LinearRegressor().smart_fit(
            list(zip(data_left["speed"], data_left["input_power"]))
        )
        regressor_right = LinearRegressor().smart_fit(
            list(zip(data_right["speed"], data_right["input_power"]))
        )

        self.log.info("Left KF value is: " + str(regressor_left.slope))
        self.log.info("Right KF value is: " + str(regressor_right.slope))

        self.log.info("Left slope value is: " + str(regressor_left.slope))
        self.log.info("Right slope value is: " + str(regressor_right.slope))

        self.log.info("Left X-int value is: " + str(regressor_left.x_intercept))
        self.log.info("Right X-int value is: " + str(regressor_right.x_intercept))

        self.log.info("Left Y-int value is: " + str(regressor_left.y_intercept))
        self.log.info("Right Y-int value is: " + str(regressor_right.y_intercept))

        self.log.info("Left KP value is: " + str(regressor_left.slope / 3))
        self.log.info("Right KP value is: " + str(regressor_right.slope / 3))

        # # Determine kP by analyzing the error
        best_kp = 0
        best_error = float("inf")
        for kp in [i / 100 for i in range(101)]:
            total_error = 0
            for i in range(len(data_left["input_power"])):
                target_speed = data_left["speed"][i]
                actual_speed = (
                        kp * data_left["input_power"][i]
                        + regressor_left.slope * data_left["input_power"][i]
                )
                error = abs(target_speed - actual_speed)
                total_error += error
            if total_error < best_error:
                best_error = total_error
                best_kp = kp

    def log_translation_discrepancy(
            self, imperial=False, tolerance=Translation1d.from_centimeters(5)
    ):
        actual_translation = self.odometry.get_translation()
        target_translation = self.target_pose.translation

        translation_discrepancy = actual_translation - target_translation

        if imperial:
            self.log.debug(
                "Target position: X: {} in Y: {} in".format(
                    *target_translation.to_inches()
                )
            )
            self.log.debug(
                "Actual position: X: {} in Y: {} in".format(
                    *target_translation.to_inches()
                )
            )
            self.log.debug(
                "ΔX: {} in ΔY: {} in".format(*(translation_discrepancy).to_inches())
            )
            self.log.debug(
                "Off by: {} in at an angle of {}°".format(
                    translation_discrepancy.length().to_inches(),
                    translation_discrepancy.angle().to_degrees(),
                )
            )
        else:
            self.log.debug(
                "Target position: X: {} cm Y: {} cm".format(
                    *target_translation.to_centimeters()
                )
            )
            self.log.debug(
                "Actual position: X: {} cm Y: {} cm".format(
                    *actual_translation.to_centimeters()
                )
            )
            self.log.debug(
                "ΔX: {} cm ΔY: {} cm".format(
                    *(translation_discrepancy).to_centimeters()
                )
            )
            self.log.debug(
                "Off by: {} cm at an angle of {}°".format(
                    translation_discrepancy.length().to_centimeters(),
                    translation_discrepancy.angle().to_degrees(),
                )
            )

        if translation_discrepancy.x_component > tolerance:
            self.log.error(
                "The robot did not move to the correct position on the X axis, check the log above"
            )
        if translation_discrepancy.y_component > tolerance:
            self.log.error(
                "The robot did not move to the correct position on the Y axis, check the log above"
            )

    # def verify_odometry(self, imperial=False, tolerance=Distance.from_centimeters(5)):
    #     self.log.trace("Entering verify_odometry")
    #     self.init()
    #
    #     self.log_translation_discrepancy(imperial, tolerance)
    #     self.log.info("First movement")
    #     self.move_distance_towards_direction_trap(Translation1d.from_feet(8), 0)
    #     self.log_translation_discrepancy(imperial, tolerance)
    #     self.log.info("Second movement")
    #     self.move_distance_towards_direction_trap(Translation1d.from_meters(0.5), -90)
    #     self.log_translation_discrepancy(imperial, tolerance)
