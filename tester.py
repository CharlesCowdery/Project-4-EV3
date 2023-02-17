#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2.motor import SpeedRPM
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.display import Display
import ev3dev2.fonts as fonts
from time import sleep

# TODO
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)

gyro = GyroSensor(INPUT_1)

screen = Display()

# this fucntion allows the user to enter in a degree value for the robot to rotate to


def turn180(deg):
    gyro.calibrate()
    gyro.reset()
    #tank_drive.on_for_rotations(SpeedPercent(50), SpeedPercent(-50), 1.889)

    while gyro.angle > deg:
        tank_drive.on(SpeedPercent(-50), SpeedPercent(50))
        screen.clear()
        screen.draw.text((10, 10), str(gyro.angle), font=fonts.load('luBS14'))
        screen.update()
        # make motors turn off
        sleep(0.1)
    tank_drive.off()

    screen.clear()
    screen.draw.text((10, 10), str(gyro.angle), font=fonts.load('luBS14'))
    screen.update()
    sleep(3)


turn180()


# method for task 2 first value is the length in cm the second is the number of laps.
# DOES NOT WORK WELL
def task2(l, n):

    # calculates the number of rotations needed to complete the task (length/distance per rotation)
    numRotations = l/13.14

    # make a loop for the number of laps
    for i in range(n):
        tank_drive.on_for_rotations(SpeedPercent(
            50), SpeedPercent(50), numRotations)
        turn180()


task2(130, 3)
