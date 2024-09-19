#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


#Variables

#]]
ev3 = EV3Brick()
leftWheelMotor = Motor(Port.D)
rightWheelMotor = Motor(Port.C)
leftButton = TouchSensor(Port.S1)
rightButton = TouchSensor(Port.S2)
reverseButton = TouchSensor(Port.S3)
buttonToggle = false

currentSpeed = 500
#]]

#Code

def MoveMotor(motor, speed, runTime):
    if runTime == 0:
        motor.run(speed)
    else:
        motor.run_time(speed, runTime)
    




##Initialize

leftWheelMotor.brake()
rightWheelMotor.brake()
ev3.speaker.beep()

while True:
    if not leftButton.pressed() and not rightButton.pressed():
        leftWheelMotor.brake()
        rightWheelMotor.brake()
    else:
        if leftButton.pressed():
            print("left button pressed")
            MoveMotor(leftWheelMotor, currentSpeed, 0)
        if rightButton.pressed():
            print("right button pressed")
            MoveMotor(rightWheelMotor, currentSpeed, 0)
    if reverseButton.pressed():
        if buttonToggle == True:
            currentSpeed = 500
            buttonToggle = False
        else:
            currentSpeed = -500
            buttonToggle = True

