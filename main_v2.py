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
from datetime import datetime
import constants as con
import navigation as nav
import devices


left_motor = devices.left_motor
right_motor = devices.right_motor
tank_drive = devices.tank_drive
gyro = devices.gyro

robot_x = 0
robot_y = 0


"""#note this is the fastest way to change motor speed
motor.speed_sp = speed
motor.run_forever() 
"""


def drive(dist,angle): #input travel distance in cm and margin in cm
    global robot_x, robot_y

    left_starting_rotation = left_motor.rotations
    right_starting_rotation = right_motor.rotations

    dist_rotation = dist/con.distance_scalar
    margin_rotation = (0)/con.distance_scalar

    speed_base = 0.2
    
    left_max_speed  = speed_base*left_motor.max_speed*0.75 #handicap
    right_max_speed = speed_base*right_motor.max_speed

    angle_array = []
    left_rotation_array = []
    right_rotation_array = []

    for i in range(0,10000):             #prepping the array so it doesnt have to resize during operations
        angle_array.append(-9999)
        left_rotation_array.append(-9999)
        right_rotation_array.append(-9999)

    index = 0

    current_angle = gyro.value(0)

    left_motor.speed_sp = left_max_speed
    right_motor.speed_sp = right_max_speed
    left_motor.command = 'run-forever'
    right_motor.command = 'run-forever'

    target_rotations = dist_rotation

    start_time = time.perf_counter()

    last_angle = current_angle

    while(True):
        current_angle = gyro.value(0)
        l_rotations = left_motor.rotations
        r_rotations = right_motor.rotations

        angle_array[index] = current_angle
        left_rotation_array[index] = l_rotations
        right_rotation_array[index] = r_rotations
        index+=1

        if(l_rotations >= target_rotations or r_rotations >=target_rotations):
            left_motor.off(True)
            right_motor.off(True)
            end_time = time.perf_counter()
            print("polling rate: {0:.1f}Hz".format(index/(end_time-start_time)))
            break

        if(last_angle != current_angle): #optimization to ensure motor angle is changed as rarely as possible
            delta_angle = (angle-current_angle)*10
            left_motor.speed_sp = left_max_speed+delta_angle
            right_motor.speed_sp = right_max_speed-delta_angle
            left_motor.command = 'run-forever'
            right_motor.command = 'run-forever'
            last_angle = current_angle

    nav.export_movement("latest.json",angle_array,left_rotation_array,right_rotation_array,index,dist,angle,start_time,end_time)
    #reconstruct(angle_array,left_rotation_array,right_rotation_array,dist,angle)

    
        



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
    nav.initialize(devices)    



main()
