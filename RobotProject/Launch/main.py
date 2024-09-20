#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


ev3 = EV3Brick()

ev3.speaker.beep()

left_motor = Motor(Port.D)
right_motor = Motor(Port.A)


obstacle_sensor = UltrasonicSensor(Port.S3)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

hammer = Motor(Port.B)

def initiate_launch():
    distanceTravelled = 0

    robot.turn(65)
    while obstacle_sensor.distance() > 140:
        robot.straight(10)
        distanceTravelled += 10
    hammer.run_angle(-5000, 200)
    robot.straight(-distanceTravelled)
    robot.turn(-65)
initiate_launch()