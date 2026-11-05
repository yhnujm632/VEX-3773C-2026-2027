import turtle
import _tkinter
import math
import time
import pathlib

from . import Constants, Pose2d

ROBOT_MAP = pathlib.Path(__file__).parent.parent.parent / "assets" / "robot_map.png"


class Canvas:
    def __init__(self):
        self.screen = turtle.Screen()
        self.screen.setup(450, 450)
        self.player = turtle.Turtle()
        try:
            print(str(ROBOT_MAP))
            self.screen.bgpic(str(ROBOT_MAP))
        except (_tkinter.TclError, turtle.Terminator):
            print("Warning: robot_map.png not found in parent Assets folder. Using a plain white background.")


    def draw_robot(self, pose: Pose2d.Pose2d):
        self.player.clear()
        self.player.color("#0f0")
        self.player.penup()





Canvas()
#g = input("sdfdf")