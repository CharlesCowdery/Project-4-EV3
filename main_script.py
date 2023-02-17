#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2.motor import SpeedRPM
from ev3dev2.display import Display
from time import sleep
import ev3dev2.fonts as fonts
import math

left_motor_port = OUTPUT_B
right_motor_port = OUTPUT_A
wheel_diameter = 3.3 #cm
wheel_circumference = 2*math.pi*wheel_diameter

left_motor = LargeMotor(left_motor_port)
right_motor = LargeMotor(right_motor_port)
"""
def driveDistance(dist,margin): #input travel distance in cm and margin in cm
    starting_rotation = (left_motor.position,right_motor.position)
    dist_rotation = dist/wheel_circumference
    margin_rotation = margin/wheel_circumference
    left_done = False
    right_done = False

    corrective_scalar = 1.0

    while(not (left_done and right_done)):
        left_rotations_remaning = left_motor.position-dist_rotation-starting_rotation[0]
        right_rotations_remaning = right_motor.position-dist_rotation-starting_rotation[1]
        if(abs(1)<margin_rotation):
            print("cock")
"""
def main():
    screen = Display()
    while(True):
        pos = right_motor.position
        screen.clear()
        screen.draw.text((10,10),str(pos),font = fonts.load('luBS14'))
        screen.update()
        sleep(1)

main()
