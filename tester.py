#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2.motor import SpeedRPM

# TODO
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)


tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)


def forward(n):
    tank_drive.on_for_rotations(SpeedPercent(50), SpeedPercent(50), n)


def backward(n):
    tank_drive.on_for_rotations(SpeedPercent(-50), SpeedPercent(-50), n)


def fullspin():
    tank_drive.on_for_rotations(SpeedPercent(50), SpeedPercent(-50), 6)
