import turtle
import _tkinter
from math import sin, cos
import time
import pathlib

#from . import Constants, Geometry
from VEXLib.Geometry.Translation1d import Translation1d
from VEXLib.Geometry.Rotation2d import Rotation2d
from VEXLib.Geometry.Translation2d import Translation2d
from VEXLib.Geometry.Pose2d import Pose2d
import src.Constants as Constants

ROBOT_MAP = pathlib.Path(__file__).parent.parent.parent / "assets" / "robot_map.png"


class Canvas:
    def __init__(self):
        self.screen = turtle.Screen()
        self.screen.setup(450, 450)
        self.player = turtle.Turtle()
        self.player.hideturtle()
        try:
            print(str(ROBOT_MAP))
            self.screen.bgpic(str(ROBOT_MAP))
        except (_tkinter.TclError, turtle.Terminator):
            print("Warning: robot_map.png not found in parent Assets folder. Using a plain white background.")

    def pause_screen(self):
        self.player.getscreen().tracer(0)

    def update_screen(self):
        self.player.getscreen().update()

    def draw_robot(self, pose: Pose2d):
        self.pause_screen()
        self.player.clear()
        self.player.color("#0f0")
        self.player.penup()

        x_position, y_position, rotation = pose.translation.x_component.to_centimeters(), pose.translation.y_component.to_centimeters(), pose.rotation.to_radians()
        width, length = Constants.DrivetrainProperties.ROBOT_WIDTH, Constants.DrivetrainProperties.ROBOT_LENGTH

        robot_corners = [
            (x_position - (cos(rotation) * (length / 2)), y_position - (sin(rotation) * (width / 2))),
        ## left off here - UNFINISHED

        ]



        self.player.goto(x_position, y_position)
        self.player.setheading(rotation)

        self.player.pendown()
        self.player.begin_fill()

        x = 1
        while x in range(1, 5):
            self.player.goto(x_position, y_position)
            x += 1

        self.player.end_fill()


        # test
        self.player.penup()
        self.player.goto(x_position, y_position)
        self.player.pendown()
        self.player.dot(20, "#000")
        self.player.penup()

        self.update_screen()






h=Canvas()
r = 0
while r in range(0, 600):
    h.draw_robot(Pose2d(Translation2d(Translation1d.from_centimeters(0), Translation1d.from_centimeters(0)), Rotation2d.from_degrees(r)))
    r += 1
    time.sleep(0.01)
g = input("sdfdf")