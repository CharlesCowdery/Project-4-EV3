#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B,OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2.motor import SpeedRPM
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.display import Display
import ev3dev2.fonts as fonts
from time import sleep
import time
import math
import random

left_motor_port = OUTPUT_C
right_motor_port = OUTPUT_B

circumference_scalar = 0.64*1.66666666
distance_scalar = 29.240853516

cross_section_length = 19.3 #cm

wheel_diameter = 3.3 #cm
wheel_circumference = 2*math.pi*wheel_diameter*circumference_scalar

left_motor = LargeMotor(left_motor_port)
right_motor = LargeMotor(right_motor_port)
tank_drive = MoveTank(left_motor_port, right_motor_port)
gyro = GyroSensor(INPUT_1)
gyro.calibrate()

robot_x = 0
robot_y = 0

def WIP_motor(speed):
    """
    motor.speed_sp = speed
    motor.run_forever() #this might not even be neccessary.
    """
    


def calibrate():
    global distance_scalar
    starting_angle = gyro.angle
    starting_left = left_motor.position/360
    starting_right = right_motor.position/360
    tank_drive.on(10,-10)
    while(gyro.angle<starting_angle+720-2):
        1+1
    tank_drive.off(None,True)
    sleep(1)
    delta_left = left_motor.position/360-starting_left
    delta_right = right_motor.position/360-starting_right
    print(delta_left)
    print(delta_right)
    print(gyro.angle)
    length = gyro.angle/360*math.pi*cross_section_length
    average_rotation = (abs(delta_left)+abs(delta_right))/2
    distance_scalar = length/average_rotation
    print(distance_scalar)
    

def drive(dist,margin,angle): #input travel distance in cm and margin in cm
    global robot_x, robot_y, distance_scalar

    left_starting_rotation = left_motor.rotations
    right_starting_rotation = right_motor.rotations

    dist_rotation = dist/distance_scalar
    margin_rotation = margin/distance_scalar

    left_done = False
    right_done = False

    drive_polarity = math.copysign(1,dist)

    speed_base = 20
    
    angle_array = []
    left_rotation_array = []
    right_rotation_array = []

    for i in range(0,10000):             #prepping the array so it doesnt have to resize during operations
        angle_array.append(-9999)
        left_rotation_array.append(-9999)
        right_rotation_array.append(-9999)

    position = 0

    while(True):
        1+1
        #Todo

def turn(deg):
    #tank_drive.on_for_rotations(SpeedPercent(50), SpeedPercent(-50), 1.889)
    print("Turning to "+str(deg)+" Degrees")
    turn_polarity = math.copysign(1, gyro.angle - deg)

    delta_angle = gyro.value()-deg
    starting_dif = delta_angle

    left_speed_max = 0.5*min(1,abs(delta_angle)/270)*left_motor.max_speed
    right_speed_max = 0.5*min(1,abs(delta_angle)/270)*right_motor.max_speed
    
    left_speed_min = 0.04*left_motor.max_speed
    right_speed_min = 0.04*right_motor.max_speed

    count = 0
    start = time.perf_counter()

    drive_percent = 0
    while turn_polarity*delta_angle > 0:
        count+=1
        left_motor.speed_sp = -turn_polarity*max(left_speed_min,left_speed_max*drive_percent)
        right_motor.speed_sp = turn_polarity*max(right_speed_min,right_speed_max*drive_percent)
        left_motor.command = 'run-forever'
        right_motor.command = 'run-forever'
        delta_angle = gyro.value()-deg
        drive_percent=abs(delta_angle/starting_dif)

    end = time.perf_counter()

    left_motor.off(True)
    right_motor.off(True)

    print("final angle = " + str(gyro.angle))
    print("Average polling rate was "+str(count/(end-start))+"Hz")
    sleep(0.5)
"""
def Task1():
    print("Task 1 Task 1 Task 1 Task 1 Task 1 Task 1 Task 1 Task 1")
    laps = 4#int(input("Enter lap number (remember a lap is there and back): "))
    distance = 90#int(input("Enter y, distance to travel (cm): "))
    #laps = random.randrange(1,6,1)
    #distance = random.randrange(10,200,10)
    print("laps = " + str(laps))
    print("distance = " + str(distance))
    for i in range(laps):
        driveDistance(distance,1,0)
        sleep(0.5)
        turn(0)
        sleep(0.5)
        driveDistance(-distance,1,0)
        sleep(1)
        turn(0)
        sleep(0.5)

def Task2():
    print("Task 2 Task 2 Task 2 Task 2 Task 2 Task 2 Task 2 Task 2")
    #laps = int(input("Enter lap number (remember a lap is there and back): "))
    degree = 0
    increment = 180
    #distance = int(input("Enter y, distance to travel (cm): "))
    laps = 3#random.randrange(1,6,1)
    distance = 120#random.randrange(10,210,10)
    print("laps = " + str(laps))
    print("distance = " + str(distance))
    for i in range(laps*2):
        driveDistance(distance,1,degree)
        degree+=increment
        sleep(0.5)
        turn(degree)
        sleep(0.5)
"""

def goToXY(x,y):
    starting_angle = 0
    driveDistance(y,0.5,starting_angle)
    turn(90)
    driveDistance(x,0.5,starting_angle+90)

def main():
    print("Calibrating!")
    gyro.calibrate()
    gyro.reset()
    print("Done!")
    gyro.mode = gyro.MODE_GYRO_ANG
    #driveDistance(30,1,0)
    #calibrate()



main()
