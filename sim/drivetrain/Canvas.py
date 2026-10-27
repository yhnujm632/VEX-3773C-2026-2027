import turtle
import _tkinter
import math
import time
import pathlib

from . import Constants


ROBOT_MAP = pathlib.Path(__file__).parent / "assets" / "robot_map.png"


class Canvas:
    def __init__(self):
        self.screen = turtle.Screen()
        self.screen.setup(450, 450)
        try:
            self.screen.bgpic(str(ROBOT_MAP))
        except (_tkinter.TclError, turtle.Terminator):
            print("Warning: robot_map.png not found in parent Assets folder. Using a plain white background.")


Canvas()