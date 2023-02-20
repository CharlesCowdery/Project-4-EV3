#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B,OUTPUT_C, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2.motor import SpeedRPM
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.display import Display
import ev3dev2.fonts as fonts
from time import sleep
import math

left_motor_port = OUTPUT_C
right_motor_port = OUTPUT_B

circumference_scalar = 0.8

wheel_diameter = 3.3 #cm
wheel_circumference = 2*math.pi*wheel_diameter*circumference_scalar

left_motor = LargeMotor(left_motor_port)
right_motor = LargeMotor(right_motor_port)
gyro = GyroSensor(INPUT_1)
gyro.calibrate()

def driveDistance(dist,margin): #input travel distance in cm and margin in cm
    gyro.calibrate()
    starting_rotation = (left_motor.position/360,right_motor.position/360) # gets wheel rotation in degrees, then converts to rotations
    dist_rotation = dist/wheel_circumference
    margin_rotation = margin/wheel_circumference
    left_done = False
    right_done = False

    corrective_scalar = 2

    starting_angle = gyro.angle

    while(not (left_done and right_done)):
        
        left_rotation_calibrated = left_motor.position/360-starting_rotation[0] # convert position to rotations, then subtract the starting rotation, to get delta rotation
        left_rotation_remaning = dist_rotation-left_rotation_calibrated        # get delta between goal rotations and current
        right_rotation_calibrated = right_motor.position/360-starting_rotation[1] 
        right_rotation_remaning = dist_rotation-right_rotation_calibrated

        
        angle = gyro.angle
        delta_angle = angle-starting_angle

        print("margin rotation = {0:.2f}, goal rotation = {1:.2f}".format(margin_rotation,dist_rotation))
        print("left rotation = {0:.2f}, right rotation = {1:.2f}".format(left_rotation_calibrated,right_rotation_calibrated))
        print("remaining left = {0:.2f}, remaining right = {0:.2f}".format(left_rotation_remaning,right_rotation_remaning))
        print()
        print("current angle: {0:.2f}, target angle: {1:.2f}".format(angle,starting_angle))

        if( not right_rotation_remaning<margin_rotation):
            speed_percent = max(min(100,100+delta_angle*corrective_scalar),-100) # when it tilts, it will slow a respective motor down to correct
            #the max and min is to make sure the input is bounded
            speed = SpeedPercent(speed_percent)
            right_motor.on(speed,False,False)
            print("right motor speed: {0:.2f}".format(speed_percent))
        else:
            right_motor.on(SpeedPercent(0),True) #brakes
            right_done = True
            
        if( not left_rotation_remaning<margin_rotation):
            speed_percent = max(min(100,100-delta_angle*corrective_scalar),-100)
            speed = SpeedPercent(speed_percent)
            left_motor.on(speed,False,False)
            print("left motor speed: {0:.2f}".format(speed_percent))

        else:
            left_motor.on(SpeedPercent(0),True) #brakes
            left_done = True
        print(str(left_done) + " " + str(right_done))
        print()

def main():
    driveDistance(100000,1)

main()
