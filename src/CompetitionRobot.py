# from ssl import PROTOCOL_SSLv23

from VEXLib.Geometry.Translation2d import Translation2d
from VEXLib.Util.time import wait_until_not, time
from VEXLib.Util.Logging import Logger, NoLogger
from VEXLib.Util.Shelf import Shelf
from DescoringArm import DescoringArm
from Constants import *
from Intake import Intake
from Vision import Vision

# This gets done first so any loggers that are created during imports use the updated index
startup_count = Shelf("logs/startup_count.csv")
startup_count.set("startup_count", startup_count.get("startup_count", -1) + 1)
if NO_LOGGING:
    robot_log = NoLogger("logs/robot")
    telemetry_log = NoLogger("logs/telemetry")
else:
    robot_log = Logger("logs/robot")
    telemetry_log = Logger("logs/telemetry")


import io
import math
import sys

import VEXLib.Math.MathUtil as MathUtil
from VEXLib.Geometry.GeometryUtil import hypotenuse
from TankDrivetrain import Drivetrain
from VEXLib.Motor import Motor
from VEXLib.Robot.RobotBase import RobotBase
from VEXLib.Robot.ScrollingScreen import ScrollingScreen
from VEXLib.Sensors.Controller import Controller
from VEXLib.Util import time, pass_function
from VEXLib.Util.Buffer import Buffer
from AutonomousRoutines import Drive, all_routines
from vex import (
    Competition,
    Color,
    FontType,
    Inertial,
    DigitalOut,
    TemperatureUnits,
    VoltageUnits,
    CurrentUnits, Optical, AiVision,
)

SmartPorts = CompetitionSmartPorts


class Robot(RobotBase):
    """
    This class contains initialization routines, controller bindings, objects representing each subsystem of the robot,
    and driver control periodic
    """
    def __init__(self, brain):
        robot_log.info("Robot __init__ called")

        super().__init__(brain)
        self.brain.screen.set_font(FontType.MONO12)

        self.controller = Controller()
        self.controller.add_deadband_step(0.05)

        self.drivetrain = Drivetrain(
            [
                Motor(
                    SmartPorts.FRONT_LEFT_DRIVETRAIN_MOTOR,
                    GearRatios.DRIVETRAIN,
                    True
                ),
                Motor(
                    SmartPorts.REAR_LOWER_LEFT_DRIVETRAIN_MOTOR,
                    GearRatios.DRIVETRAIN,
                    True,
                ),
                Motor(
                    SmartPorts.REAR_UPPER_LEFT_DRIVETRAIN_MOTOR,
                    GearRatios.DRIVETRAIN,
                    False,
                ),
            ],
            [
                Motor(
                    SmartPorts.FRONT_RIGHT_DRIVETRAIN_MOTOR,
                    GearRatios.DRIVETRAIN,
                    False,
                ),
                Motor(
                    SmartPorts.REAR_LOWER_RIGHT_DRIVETRAIN_MOTOR,
                    GearRatios.DRIVETRAIN,
                    False,
                ),
                Motor(
                    SmartPorts.REAR_UPPER_RIGHT_DRIVETRAIN_MOTOR,
                    GearRatios.DRIVETRAIN,
                    True,
                ),
            ],
            Inertial(SmartPorts.INERTIAL_SENSOR),
            Vision(AiVision(SmartPorts.VISION_SENSOR, DrivetrainProperties.LONG_GOAL_COLOR_DESC))
        )

        self.calibrate_sensors()

        self.intake = Intake(
            Motor(SmartPorts.LEVER_MOTOR, GearRatios.LEVER_MOTOR, True),
            Motor(SmartPorts.FLOATING_INTAKE_MOTOR, GearRatios.FLOATING_INTAKE, False),
            DigitalOut(ThreeWirePorts.HOOD_SOLENOID),
            DigitalOut(ThreeWirePorts.RAISE_SOLENOID),
            Optical(SmartPorts.OPTICAL_SENSOR)
        )

        self.descoring_arm = DescoringArm(
            DigitalOut(ThreeWirePorts.DESCORING_ARM_SOLENOID_OUT),
            DigitalOut(ThreeWirePorts.DESCORING_ARM_SOLENOID_UP)
        )

        self.screen = ScrollingScreen(self.brain.screen, Buffer(20))
        self.alliance_color = None

        self.user_preferences = DefaultPreferences

        self.competition = Competition(self.on_driver_control, self.on_autonomous)

        self.available_autonomous_routines = all_routines
        self.selected_autonomous = Drive(self)

        self.brain.screen.pressed(self.flush_all_logs)

        self.setup_complete = False
        self.iteration_count = 0

    def flush_all_logs(self, message="Screen pressed; flushing logs manually"):
        robot_log.info("Flushing all logs")
        for log in [robot_log, self.drivetrain.log, self.drivetrain.debug_log]:
            log.info(message)
            log.flush_logs()

    def log_and_print(self, *parts):
        self.brain.screen.set_font(FontType.MONO15)
        message = " ".join(map(str, parts))
        self.screen.print(message)
        robot_log.log(message)
        print(message)

    def start(self):
        robot_log.info("Robot start called")
        try:
            self.on_setup()
        except Exception as e:
            exception_buffer = io.StringIO()
            sys.print_exception(e, exception_buffer)
            for log_entry in exception_buffer.getvalue().split("\n"):
                robot_log.fatal(str(log_entry))
            robot_log.flush_logs()
            raise e

    @robot_log.logged
    def on_setup(self):
        robot_log.info("Robot on_setup called")
        robot_log.info("Selecting autonomous and drive style")
        robot_log.trace(
            "Available autonomous routines:", self.available_autonomous_routines
        )
        selected_auto, drive_style, alliance_color = self.controller.get_multiple_selections([[auto.name for auto in self.available_autonomous_routines], ["Ross", "Debug"], ["red", "blue"]])
        if alliance_color == "blue":
            self.alliance_color = Color.BLUE
        else:
            self.alliance_color = Color.RED

        for autonomous in self.available_autonomous_routines:
            if selected_auto == autonomous.name:
                self.selected_autonomous = autonomous(self)
                break

        if isinstance(self.selected_autonomous, Drive):
            robot_log.warn("DoNothingAutonomous autonomous routine selected")

        self.log_and_print("Selected autonomous routine:", selected_auto)
        self.selected_autonomous.pre_match_setup()

        self.log_and_print("Selected drive style:", drive_style)

        self.align_robot()

        if drive_style == "Ross":
            self.user_preferences = RossPreferences
            self.setup_default_bindings()
        elif drive_style == "Debug":
            self.user_preferences = DebugPreferences
            self.setup_debug_bindings()

        self.log_and_print("Set up user preferences:", drive_style)

        robot_log.info("Setup complete")
        robot_log.debug("Unlocking setup lock")

        self.setup_complete = True

    @robot_log.logged
    def calibrate_sensors(self):
        robot_log.info("Calibrating sensors")
        # Set initial sensor positions and calibrate mechanisms.

        robot_log.debug("Calibrating inertial sensor")
        self.drivetrain.odometry.inertial_sensor.calibrate()
        while self.drivetrain.odometry.inertial_sensor.is_calibrating():
            time.sleep_ms(5)
        robot_log.debug("Calibrated inertial sensor successfully")
        time.sleep(2)
        self.controller.rumble("..")

    @robot_log.logged
    def align_robot(self):
        robot_log.info("Aligning robot")
        robot_log.info("Waiting for robot alignment")
        target_rotation = self.selected_autonomous.startup_angle()
        self.controller.rumble("--")

        self.log_and_print("Please line up robot...")
        self.log_and_print("The screen will turn green when properly aligned")

        while not self.controller.buttonA.pressing():
            # Get the current rotation as a Rotation2d object.
            current_rotation = self.drivetrain.odometry.get_rotation()

            # Initialize variables for the minimum signed error.
            min_signed_error = None
            min_abs_error = float("inf")
            # Calculate the signed error for each target and find the minimum absolute error.
            error_rotation = (current_rotation - target_rotation).normalize()
            error_deg_signed = error_rotation.to_degrees()
            abs_error = abs(error_deg_signed)
            if abs_error < min_abs_error:
                min_abs_error = abs_error
                min_signed_error = error_deg_signed

            # Decide on the screen color based on the absolute error.
            if min_abs_error > 5.0:
                # More than 5° away: flash the screen.
                flash_period_ms = 500
                current_time = time.time_ms()  # Get current time in ms.
                if ((current_time // flash_period_ms) % 2) == 0:
                    screen_color = Color.RED
                    text_color = Color.BLACK
                else:
                    screen_color = Color.BLACK
                    text_color = Color.RED
            else:
                # Within 5°: interpolate hue from green (120° at 0° error) to red (0° at 5° error).
                ratio = min_abs_error / 5.0
                hue = MathUtil.interpolate(120, 0, ratio, allow_extrapolation=False)
                screen_color = Color().hsv(hue, 1, 1)
                text_color = Color.BLACK

            # Do all logic before drawing to the screen to prevent flashing
            # Determine direction based on the signed error.
            if min_signed_error >= 0:
                direction = "CW"
            else:
                direction = "CCW"

            # Display the absolute error (formatted to two decimals) and the direction.
            display_error = abs(min_signed_error)
            display_text = "{:.2f}° {}".format(display_error, direction)

            self.brain.screen.set_fill_color(screen_color)
            self.brain.screen.set_pen_color(screen_color)
            self.brain.screen.draw_rectangle(0, 0, 480, 240)

            self.brain.screen.set_font(FontType.MONO60)
            self.brain.screen.set_cursor(2, 4)
            self.brain.screen.set_pen_color(text_color)

            self.brain.screen.print(display_text)

            if min_abs_error < 0.25:
                self.brain.screen.set_cursor(3, 3)
                self.brain.screen.print("Lined Up :)")
            else:
                self.brain.screen.set_cursor(1, 2)
                self.brain.screen.print("Please Rotate")

            self.drivetrain.update_odometry()
            time.sleep_ms(20)

        # Reset colors
        self.brain.screen.set_fill_color(Color.BLACK)
        self.brain.screen.set_pen_color(Color.WHITE)

        # Final update and logging after button press.
        self.drivetrain.update_odometry()
        final_deg = self.drivetrain.odometry.get_rotation_normalized().to_degrees()
        self.brain.screen.clear_screen()
        self.log_and_print("Lined up, current angle:", final_deg)
        wait_until_not(self.controller.buttonA.pressing)

    @robot_log.logged
    def on_driver_control(self):
        self.intake.stop_floating_intake()
        self.selected_autonomous.cleanup()
        self.flush_all_logs("Flushing logs before driver control")
        self.drivetrain.left_drivetrain_PID.pid_gains = self.user_preferences.PIDF_GAINS_LEFT_DRIVER
        self.drivetrain.right_drivetrain_PID.pid_gains = self.user_preferences.PIDF_GAINS_RIGHT_DRIVER
        robot_log.trace("Driver control started")
        self.intake.raise_intake()
        while not self.setup_complete and self.competition.is_driver_control():
            time.sleep_ms(20)
        self.brain.screen.clear_screen()
        while True:
            self.driver_control_periodic()
            time.sleep_ms(10)

    @robot_log.logged
    def on_autonomous(self):
        self.intake.stop_floating_intake()
        self.drivetrain.left_drivetrain_PID.pid_gains = self.user_preferences.PIDF_GAINS_LEFT_AUTO
        self.drivetrain.right_drivetrain_PID.pid_gains = self.user_preferences.PIDF_GAINS_RIGHT_AUTO
        self.drivetrain.odometry.pose.translation = Translation2d()
        self.drivetrain.target_pose = self.drivetrain.odometry.pose
        self.drivetrain.rotation_PID.setpoint = self.drivetrain.target_pose.rotation.to_radians()
        self.selected_autonomous.execute()

    def log_telemetry(self):
        self.iteration_count += 1
        if self.iteration_count % 100 == 0:
            telemetry_log.info("Telemetry at iteration", self.iteration_count)
            telemetry_log.info("Time: ", time.time())
            telemetry_log.info("Drivetrain Pose:", self.drivetrain.odometry.pose)
            telemetry_log.info("Left Motor temps:", [motor.temperature(TemperatureUnits.CELSIUS) for motor in self.drivetrain.left_motors])
            telemetry_log.info("Right Motor temps:", [motor.temperature(TemperatureUnits.CELSIUS) for motor in self.drivetrain.right_motors])
            telemetry_log.info("Battery Voltage:", self.brain.battery.voltage(VoltageUnits.VOLT))
            telemetry_log.info("Battery Temperature:", self.brain.battery.temperature(TemperatureUnits.CELSIUS))
            telemetry_log.info("Battery Current:", self.brain.battery.current(CurrentUnits.AMP))
            telemetry_log.info("Battery Capacity:", self.brain.battery.capacity())
            telemetry_log.flush_logs()

    def driver_control_periodic(self):
        self.log_telemetry()
        if self.controller.buttonDown.pressing():
            i = int(self.iteration_count / 1) % 94
            self.brain.screen.draw_image_from_file("assets/output_frame_" + ("0" * (4 - len(str(i))) + str(i)) + "-fs8.png", 0, 0)
        left_speed, right_speed = self.controller.get_wheel_speeds(self.user_preferences.CONTROL_STYLE)

        if self.user_preferences.ENABLE_DRIVING:
            if self.user_preferences.USE_PIDF_CONTROL:
                self.drivetrain.set_speed_zero_to_one(left_speed, right_speed)
                self.drivetrain.update_powers()
            else:
                self.drivetrain.set_powers(
                    left_speed * self.user_preferences.MOVE_SPEED,
                    right_speed * self.user_preferences.MOVE_SPEED,
                )

        self.drivetrain.update_odometry()
        left_stick_x = self.controller.left_stick_x()
        left_stick_y = self.controller.left_stick_y()
        left_stick_x_processed, left_stick_y_processed = left_stick_x, left_stick_y

        if hypotenuse(left_stick_x_processed, left_stick_y_processed) > 1:
            angle = math.atan2(left_stick_y_processed, left_stick_x_processed)
            left_stick_x_processed = math.cos(angle)
            left_stick_y_processed = math.sin(angle)

        if self.user_preferences.INPUT_DEBUG_MODE:
            self.brain.screen.set_pen_color(Color.RED)
            self.brain.screen.draw_pixel(left_stick_x * 100 + 120, -left_stick_y * 100 + 120)
            self.brain.screen.set_pen_color(Color.GREEN)
            self.brain.screen.draw_pixel(left_stick_x_processed * 100 + 360, -left_stick_y_processed * 100 + 120)


    @robot_log.logged
    def setup_default_bindings(self):
        robot_log.info("Setting up default controller bindings")


    @robot_log.logged
    def setup_debug_bindings(self):
        robot_log.info("Setting up debug controller bindings")
        self.setup_default_bindings()
        self.controller.buttonX.pressed(self.drivetrain.verify_speed_pid)
        self.controller.buttonA.pressed(lambda: self.drivetrain.measure_properties(True))
        self.controller.buttonUp.pressed(lambda: self.drivetrain.turn_to(Rotation2d.from_degrees(0)))
        self.controller.buttonLeft.pressed(lambda: self.drivetrain.turn_to(Rotation2d.from_degrees(90)))
        self.controller.buttonDown.pressed(lambda: self.drivetrain.turn_to(Rotation2d.from_degrees(180)))
        self.controller.buttonRight.pressed(lambda: self.drivetrain.turn_to(Rotation2d.from_degrees(270)))
