from vex import DigitalOut

class PistonActuator:
    """
    A subsystem class that controls a pneumatic piston for the mid-goal hood actuator.

    Attributes:
        piston (DigitalOut): A digital output object controlling the pneumatic solenoid.
    """

    def __init__(self, piston: DigitalOut):
        """
        Initializes the MatchLoadHelper with a specified piston output.

        Args:
            piston (DigitalOut): The digital output that activates the pneumatic piston.
        """
        self.piston = piston
        self._state = False

    def extend(self):
        """
        Extends the piston to activate the match loader mechanism.

        This method sends a high (True) signal to the digital output,
        opening the pneumatic valve and extending the piston.
        """
        self._state = True
        self.piston.set(self._state)

    def retract(self):
        """
        Retracts the piston to deactivate the match loader mechanism.

        This method sends a low (False) signal to the digital output,
        closing the pneumatic valve and retracting the piston.
        """
        self._state = False
        self.piston.set(self._state)

    def toggle(self):
        """
        Toggles the state of the piston between extended and retracted.

        This method switches the current state of the piston and updates
        the digital output accordingly.
        """
        self._state = not self._state
        self.piston.set(self._state)
