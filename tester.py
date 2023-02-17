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


def forward(n):
    tank_drive.on_for_rotations(SpeedPercent(50), SpeedPercent(50), n)


def backward(n):
    tank_drive.on_for_rotations(SpeedPercent(-50), SpeedPercent(-50), n)


def turn180():
    gyro.calibrate()

    gyro.reset()

    tank_drive.on_for_rotations(SpeedPercent(50), SpeedPercent(-50), 1.75)

    screen.draw.text((10, 10), str(gyro.angle), font=fonts.load('luBS14'))

    screen.update()
    sleep(3)


turn180()
