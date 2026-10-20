from VEXLib.Math import MathUtil
from VEXLib.Motor import Motor
from VEXLib.Util import time
from Constants import IntakeConstants
from vex import DigitalOut, TorqueUnits, PERCENT, Color, Optical, Thread, FORWARD, HOLD, COAST, DEGREES, LedStateType


class Intake:
    def __init__(self, elevator_motor_1: Motor, elevator_motor_2: Motor, intake_motor: Motor):
        self.elevator_motor_1 = elevator_motor_1
        self.elevator_motor_2 = elevator_motor_2
        self.intake_motor = intake_motor
        self.elevator_target = 0
        self.elevator_speed = 0
        self.time_updated_setpoint = 0
        self.elevator_step_amount = 20
        self.set_elevator_velocity(0)

    def set_elevator_velocity(self, velocity):
        self.elevator_motor_1.spin(FORWARD)
        self.elevator_motor_2.spin(FORWARD)
        self.elevator_motor_1.set_velocity(velocity)
        self.elevator_motor_2.set_velocity(velocity)

    def get_elevator_position(self):
        return (self.elevator_motor_1.position(DEGREES) + self.elevator_motor_2.position(DEGREES)) / 2

    def move_elevator_to_position(self, position_deg):
        self.elevator_motor_1.set_velocity(IntakeConstants.RETURN_SPEED, PERCENT)
        self.elevator_motor_2.set_velocity(IntakeConstants.RETURN_SPEED, PERCENT)
        self.elevator_motor_1.spin_to_position(position_deg, DEGREES)
        self.elevator_motor_2.spin_to_position(position_deg, DEGREES)

    def move_elevator_down(self):
        self.set_elevator_speed(100)
        self.set_elevator_setpoint(0)

    def set_elevator_setpoint(self, setpoint):
        self.elevator_target = setpoint
        self.time_updated_setpoint = time.time()

    def set_elevator_speed(self, speed):
        self.elevator_speed = speed

    def run_intake(self, speed = 1):
        """Run the intake motor at a specified speed."""
        self.intake_motor.set(speed)

    def stop_intake(self):
        """Stop the intake motor"""
        self.intake_motor.set(0)

    def step_up(self):
        self.set_elevator_setpoint(self.elevator_target + self.elevator_step_amount)

    def lever_is_stalled(self):
        # Stall if torque is high and velocity is low
        torque_threshold = 1  # Adjust as needed
        velocity_threshold = 10  # Adjust as needed

        is_stalled = (
                self.elevator_motor_1.get() != 0 and
                self.elevator_motor_1.torque(TorqueUnits.NM) > torque_threshold and
                abs(self.elevator_motor_1.velocity(PERCENT)) < velocity_threshold
         )

        current_time = time.time()
        if not is_stalled:
            self.last_not_stalled_timestamp = current_time

        # Only report stall if the condition has persisted for at least 0.1s
        stalled_duration = current_time - self.last_not_stalled_timestamp
        return is_stalled and stalled_duration > 0.1

    def periodic(self):
        error = self.elevator_target - ((self.elevator_motor_1.position(DEGREES) + self.elevator_motor_2.position(DEGREES)) / 2)
        if MathUtil.is_near(self.elevator_target, ((self.elevator_motor_1.position(DEGREES) + self.elevator_motor_2.position(DEGREES)) / 2), 3):
            self.set_elevator_velocity(0)
        else:
            self.set_elevator_velocity(MathUtil.clamp(self.elevator_speed * error * 0.1, -self.elevator_speed, self.elevator_speed))

        # Check if we should return to lowered setpoint
        # If we are

        # time_since_setpoint_changed = time.time() - self.time_updated_setpoint
        #
        # if self.lever_target != 0 and time_since_setpoint_changed > 1:
        #     self.set_lever_setpoint(0)
