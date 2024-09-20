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




robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)


obstacle_sensor = UltrasonicSensor(Port.S3)
color_sensor = ColorSensor(Port.S2)
gyro_sensor = GyroSensor(Port.S4)
spinny_thing = Motor(Port.C)

def set_up_satalite():
    robot.straight(285)
    robot.turn(-50)
    robot.straight(200)
    robot.straight(-200)
    robot.turn(50)
    robot.straight(-285)

def rescue_astronauts():
    got_astro = False
    
    robot.turn(-15)
    while True:
        print(obstacle_sensor.distance())
        if got_astro == False:
            while obstacle_sensor.distance() < 75:
                if color_sensor.color() == Color.BLACK:
                    #rotate towards the white lego plate
                    robot.run_time(-100, 100)
                    robot.turn(90)
                    robot.run_time(100, 100)
                    robot.turn(-90)

                elif color_sensor.color() == Color.WHITE:
                    spinny_thing.run_angle(200, 90)
                    got_astro = True
                else:
                    robot.turn(20)
            robot.straight(20)
        else:
            gyro_sensor.reset_angle(0)
            robot.straight(20)
            if color_sensor.color() == None:
                spinny_thing.run_angle(200, -100)
                break

def kill_other_astro(colour):
    robot.straight(830)
    pass

def rescue_astronauts_V2_black():
    on_left = True
    distance = 0
    got_astro = False
    robot.turn(-22)
    robot.straight(830)
    robot.turn(18)
    robot.straight(20)
    while got_astro == False:
        if color_sensor.color() == Color.BLACK:
            robot.straight(40)
            spinny_thing.run_angle(100, 70)
            robot.straight(-40)
            got_astro = True
        elif color_sensor.color() == Color.WHITE:
            robot.straight(-20)
            robot.turn(-90)
            robot.straight(40)
            on_left = false

    if on_left:
        robot.straight(-60)
        robot.turn(-30)

    robot.straight(-860)
    robot.turn(180)
    spinny_thing.run_angle(-100, 90)
    robot.straight(-100)



def rescure_MSL_robot():
    robot.straight(100)
    spinny_thing.run_angle(200, -100)
    robot.straight(-100)
    pass

def launch_the_satalite():
    robot.straight(100)
    spinny_thing.run_angle(200, -100)
    robot.straight(-100)


def return_rock_samples():
    pass

def secure_power_supply():
    robot.straight(100)
    spinny_thing.run_angle(200, -100)
    robot.straight(-100)

def initiate_launch():
    hammer.run_angle(1400, 130)

def test():
    spinny_thing.run_angle(100, 90)
    spinny_thing.run_angle(-100, 90)
    while True:
        #print(gyro_sensor.angle())
        #while obstacle_sensor.distance() < 300:
        #    robot.turn(20)
        print(color_sensor.color(), obstacle_sensor.distance())
        #robot.straight(20)
    while True:
        print(distance_travelled)
        if distance_travelled < 400:
            # Begin driving forward at 200 millimeters per second.
            robot.drive(200, 0)
            steps["drive"] = 200
            distance_travelled += 200
            

            # Wait until an obstacle is detected. This is done by repeatedly
            # doing nothing (waiting for 10 milliseconds) while the measured
            # distance is still greater than 300 mm.
            """while obstacle_sensor.distance() > 30:
                robot.turn(120)
                steps["rotate"] = 120"""
        else:
            for step in steps:
                if step == "drive":
                    robot.drive(-steps[step], 0)
                elif step == "rotate":
                    robot.turn(-steps[step])
            break

rescue_astronauts_V2_black()