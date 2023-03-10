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
distance_scalar = 27.8

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
    

def driveDistance(dist,margin,angle): #input travel distance in cm and margin in cm
    global robot_x, robot_y, distance_scalar
    robot_x += math.cos(angle/180*math.pi)*dist
    robot_y += math.sin(angle/180*math.pi)*dist
    starting_rotation = (left_motor.position/360,right_motor.position/360) # gets wheel rotation in degrees, then converts to rotations
    dist_rotation = dist/distance_scalar
    margin_rotation = margin/distance_scalar
    left_done = False
    right_done = False

    drive_polarity = math.copysign(1,dist)

    speed_base = 20
    left_modifier = 0
    right_modifier = 0.5

    corrective_scalar = 1

    starting_angle = angle

    left_rotation_calibrated = 0
    left_rotation_remaning = 0
    right_rotation_calibrated = 0
    right_rotation_remaning = 0

    angle = 0
    delta_angle = 0

    right_speed = 0
    left_speed = 0


    while(not (left_done and right_done)):
        
        left_rotation_calibrated = left_motor.position/360-starting_rotation[0] # convert position to rotations, then subtract the starting rotation, to get delta rotation
        left_rotation_remaning = dist_rotation-left_rotation_calibrated        # get delta between goal rotations and current
        right_rotation_calibrated = right_motor.position/360-starting_rotation[1] 
        right_rotation_remaning = dist_rotation-right_rotation_calibrated

        angle = gyro.angle
        delta_angle = angle-starting_angle

        #print("margin rotation = {0:.2f}, goal rotation = {1:.2f}".format(margin_rotation,dist_rotation))
        #print("left rotation = {0:.2f}, right rotation = {1:.2f}".format(left_rotation_calibrated,right_rotation_calibrated))
        #print("remaining left = {0:.2f}, remaining right = {0:.2f}".format(left_rotation_remaning,right_rotation_remaning))
        #print()
        #print("current angle: {0:.2f}, target angle: {1:.2f}".format(angle,starting_angle))

        if( not abs(right_rotation_remaning)<margin_rotation):
            speed_percent = max(min(100,
                                    (speed_base*right_rotation_remaning/dist_rotation+right_modifier)*drive_polarity+delta_angle*corrective_scalar
                                    ),10) # when it tilts, it will slow a respective motor down to correct
            #the max and min is to make sure the input is bounded
            right_speed = SpeedPercent(speed_percent)
            #print("right motor speed: {0:.2f}".format(speed_percent))
        else:
            right_speed = SpeedPercent(0)
            right_motor.on(SpeedPercent(0),True) #brakes
            right_done = True
            
        if( not abs(left_rotation_remaning)<margin_rotation):
            speed_percent = max(min(100,
                                    (speed_base*left_rotation_remaning/dist_rotation+left_modifier)*drive_polarity-delta_angle*corrective_scalar
                                    ),10)
            left_speed = SpeedPercent(speed_percent)
            #print("left motor speed: {0:.2f}".format(speed_percent))

        else:
            left_speed = SpeedPercent(0)
            left_motor.on(SpeedPercent(0),True) #brakes
            left_done = True
        #print(str(left_done) + " " + str(right_done))
        #print()
        right_motor.on(right_speed,right_done,False)
        left_motor.on(left_speed,left_done,False)
    right_motor.on(0,True, False)
    left_motor.on(0,True,False)

#this function makes the robot turn to a specified absolute angle
def turn(deg):
    print("Turning to "+str(deg)+" Degrees")

    #gets the direction the motors need to spin
    turn_polarity = math.copysign(1, gyro.angle - deg)

    #gets inital angle delta
    delta_angle = gyro.value()-deg
    starting_dif = delta_angle #saves the initial delta

    #defines the maximum speed the motor can reach in most normal circumstances
    #its set to scale so larger angle deltas have a higher top speed to a certain point
    #the front decimal is the percent speed. It can be set higher, but you risk losing precision
    left_speed_max = 0.5*min(1,abs(delta_angle)/270)*left_motor.max_speed  
    right_speed_max = 0.5*min(1,abs(delta_angle)/270)*right_motor.max_speed
    
    #this is set so it doesnt try to turn at like, 1% speed towards the end
    left_speed_min = 0.04*left_motor.max_speed
    right_speed_min = 0.04*right_motor.max_speed

    #just some benchmarking stuff
    count = 0
    start = time.perf_counter()

    #defines the motor speed
    drive_percent = 0

    #this is set up so while it is before the angle its positive, and the minute it goes past its mark it goes negative
    while turn_polarity*delta_angle > 0:
        count+=1 #increment
        left_motor.speed_sp = -turn_polarity*max(left_speed_min,left_speed_max*drive_percent)
        right_motor.speed_sp = turn_polarity*max(right_speed_min,right_speed_max*drive_percent) #this sets the speed the motor will run at
        left_motor.command = 'run-forever'
        right_motor.command = 'run-forever' #this tells the motor to update its speed
        delta_angle = gyro.value()-deg
        drive_percent=abs(delta_angle/starting_dif)

    end = time.perf_counter()

    left_motor.off(True) #makes the motor brake
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
    driveDistance(y,0.3,starting_angle)
    turn(90)
    driveDistance(x,0.3,starting_angle+90)
    turn(starting_angle+90)

def testing_set1():
    global robot_x, robot_y
    print("You have selected test suite 1.")
    while(True):
        distance = float(input("enter distance to travel in inches: "))*2.54 #convert to cm
        test_count = 0
        while(True):
            user_input = input("enter cancel to leave, otherwise hit enter to continue: ")
            test_count += 1

            if(user_input == "cancel"):
                break
            print("Test "+str(test_count)+":")

            gyro.reset()
            robot_x = 0
            robot_y = 0

            driveDistance(distance,0.3,0)
            turn(0)
        print("Ok! returning to distance select.")

def testing_set2():
    global robot_x, robot_y
    print("You have selected test suite 2.")
    while(True):
        distance_x = float(input("enter distance of second leg in inches: "))*2.54 #convert to cm
        distance_y = 12*2.54
        test_count = 0
        while(True):
            user_input = input("enter stop to leave, otherwise hit enter to continue: ")
            test_count += 1
            
            if(user_input == "cancel"):
                break

            print("Test "+str(test_count)+":")


            gyro.reset()
            robot_x = 0
            robot_y = 0
            goToXY(distance_x,distance_y)
        print("Ok! returning to distance select.")
            




def main():
    global distance_scalar 
    gyro.reset()
    gyro.mode = gyro.MODE_GYRO_ANG
    #driveDistance(30,1,0)
    #calibrate()
    testing_set1()

distance_scalar = 28.2 #if it overshoots raise this value. vice versa

sleep(1)
gyro.calibrate()
main()
