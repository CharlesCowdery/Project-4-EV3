#!/usr/bin/env python3
#from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B,OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank
#from ev3dev2.sensor import INPUT_1
#from ev3dev2.sensor.lego import TouchSensor
#from ev3dev2.led import Leds
#from ev3dev2.sound import Sound
#from ev3dev2.motor import SpeedRPM
#from ev3dev2.sensor.lego import GyroSensor
#from ev3dev2.display import Display
import ev3dev2.fonts as fonts
from time import sleep
import time
import math
from datetime import datetime
import constants_r as con
import navigation_r as nav
import devices_r as devices



left_motor = devices.left_motor
right_motor = devices.right_motor
tank_drive = devices.tank_drive
gyro = devices.gyro
color_reader = devices.color_reader

robot_x = 0
robot_y = 0
my_heading = 0


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

    speed_base = -0.6
    
    left_max_speed  = speed_base*left_motor.max_speed#*0.75 #handicap
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
        l_rotations = left_motor.rotations-left_starting_rotation
        r_rotations = right_motor.rotations-right_starting_rotation

        angle_array[index] = current_angle
        left_rotation_array[index] = l_rotations
        right_rotation_array[index] = r_rotations
        index+=1

        if(-l_rotations >= target_rotations or -r_rotations >=target_rotations):
            left_motor.off(True)
            right_motor.off(True)
            end_time = time.perf_counter()
            print("polling rate: {0:.1f}Hz".format(index/(end_time-start_time)))
            break

        if(last_angle != current_angle): #optimization to ensure motor angle is changed as rarely as possible
            delta_angle = (angle-current_angle)*10
            left_motor.speed_sp = left_max_speed-delta_angle
            right_motor.speed_sp = right_max_speed+delta_angle
            left_motor.command = 'run-forever'
            right_motor.command = 'run-forever'
            last_angle = current_angle

        if(devices.ultrasonic.distance_centimeters<10):
            left_motor.off(True)
            right_motor.off(True)
            avg = 0
            while(avg < 10):
                readings = 0
                for i in range(100):
                    readings+=(devices.ultrasonic.distance_centimeters)
                avg = readings/100
            last_angle=-9999


    nav.export_movement("latest.json",angle_array,left_rotation_array,right_rotation_array,index,dist,angle,start_time,end_time)
    #print(nav.predict_change(angle_array,left_rotation_array,right_rotation_array))
    #reconstruct(angle_array,left_rotation_array,right_rotation_array,dist,angle)


def turn(deg):
    #tank_drive.on_for_rotations(SpeedPercent(50), SpeedPercent(-50), 1.889)
    print("Turning to "+str(deg)+" Degrees")
    turn_polarity = math.copysign(1, gyro.angle - deg)

    delta_angle = gyro.value()-deg
    starting_dif = delta_angle

    left_speed_max = 1*min(1,abs(delta_angle)/270)*left_motor.max_speed
    right_speed_max = 1*min(1,abs(delta_angle)/270)*right_motor.max_speed
    
    left_speed_min = 0.1*left_motor.max_speed
    right_speed_min = 0.1*right_motor.max_speed

    count = 0
    start = time.perf_counter()

    drive_percent = 0
    while turn_polarity*delta_angle > 0:
        count+=1
        left_motor.speed_sp = turn_polarity*max(left_speed_min,left_speed_max*drive_percent)
        right_motor.speed_sp = -turn_polarity*max(right_speed_min,right_speed_max*drive_percent)
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

def box_protocol(relative_angle):
    code = read_barcode(relative_angle, target_dist=3)
    drive_simple(3.5,10)
    turn(relative_angle-90)
    box_handling()
    drive_simple(-code[1],10)
    turn(relative_angle)

def read_barcode(relative_angle=0,target_dist=2,white_threshold = 20):
    tank_drive.on(-10,-10)
    while(devices.color_reader.reflected_light_intensity < 1):
        1+1
    tank_drive.off()

    drive_simple(10,10)
    turn(relative_angle-90)
    dist = 0
    for i in range(1000):
        dist+=devices.ultrasonic.distance_centimeters
    dist/=1000
    drive_simple(dist+3,10)
    drive_simple(-target_dist,10)
    turn(relative_angle)

    drive_simple(-15,10)

    tank_drive.on(-10,-10)
    while(color_reader.color!=2):
        1+1
    tank_drive.off()

    drive_simple(-0.5,10)

    colors = ""
    color_arr = []
    for i in range(4):
        #colors[i] = devices.color_reader.color != 2
        colors += str(devices.color_reader.reflected_light_intensity > white_threshold)
        color_arr.append(devices.color_reader.reflected_light_intensity > white_threshold)
        drive_simple(1.32,10)
    
    print(colors)

    color_catagories = dict()
    color_catagories["FalseTrueTrueTrue"] = 0
    color_catagories["FalseTrueFalseTrue"] = 1
    color_catagories["FalseFalseTrueTrue"] = 2
    color_catagories["FalseTrueTrueFalse"] = 3

    print(colors)    
    return [color_catagories[colors],dist+3-target_dist,color_arr]


def drive_simple(dist,speed):
    tank_drive.on_for_rotations(speed,speed,-dist/con.distance_scalar)

def box_handling():
    devices.ultrasonic.distance_centimeters_continuous
    dist = devices.ultrasonic.distance_centimeters
    if(dist == 255): dist = 0
    print(dist)
    drive_simple((dist),25)
    sleep(0.5)
    nav.home_lift(15,25)
    sleep(0.5)

    drive_simple(-4,25)
    sleep(0.5)

    drive_simple(5,10)
    sleep(0.5)

    nav.raise_lift(True)

    drive_simple(-1,10)


def release_box():
    1+1

def navToBox(box_x,box_y):
    global robot_x, robot_y, my_heading

    relative_angle = 0

    delta_y = abs(robot_y-box_y)

    robot_y = box_y

    drive(delta_y*2.54,0)

    turn(90)

    my_heading=90

    delta_x = abs(robot_x-box_x)

    drive(delta_x*2.54,my_heading)

    robot_x=box_x

def positionToBox(box_above):
    global my_heading

    if(not box_above):
        my_heading=270
        turn(my_heading)
        
    drive_simple(-10,10)
        
    
def mainTask(shelf,box_number,goal_letter, box_id=0):
    global robot_x, robot_y, my_heading

    shelf_letter = shelf[0].lower()
    shelf_number = int(shelf[1])-1

    goal_letter=goal_letter.lower()

    if(shelf_letter == "a" or shelf_letter == "c"):
        column = 0
    else:
        column = 1

    if(shelf_letter == "a" or shelf_letter == "b"):
        row = 0
    else:
        row = 2
    
    row+=shelf_number

    row_position = (box_number-1)%6

    is_top = box_number > 6


    goal_positions = [[6,-6],[102,-6],[6,114],[102,114]]


    goals = ["a","b","c","d"]

    starting_spot = 0#goals.index(starting_letter)
    goal = goals.index(goal_letter)

    starting_x = goal_positions[starting_spot][0]
    starting_y = goal_positions[starting_spot][1]

    robot_x = starting_x
    robot_y = starting_y

    relative_angle = 0

    box_x = 12+column*48+(3+row_position*6)
    box_y = 6+(24 if is_top else 0) +row*24
    box_y_real = box_y

    box_y+=(1.2 if is_top else -1.2)

    goal_x = goal_positions[goal][0]
    goal_y = goal_positions[goal][1]


    navToBox(box_x,box_y)
    positionToBox(not is_top)
    print("Reading barcode")
    code = read_barcode(target_dist=3, relative_angle=my_heading)

    correct_box = code[0]==box_id


    if(not correct_box):
        barcode_string = ""
        for color in code[2]:
            barcode_string+=("O" if color else "#")
        final_string = "incorrect box: {} ({})!\n location is ({},{})\n returning home".format(barcode_string,code[0]+1,box_x,box_y_real)
        print(final_string)

        devices.display.text_pixels(final_string,x=0,y=32,font=fonts.load('luBS14'))
        devices.display.update()

        if is_top:
            if my_heading == 270:
                my_heading = 360
            else:
                my_heading = 0
        else:
            my_heading = 180

        turn(my_heading)

        drive_simple(4.54,10)

        my_heading = 270

        turn(my_heading)

        #drive_simple(-code[1],10)
        delta_start_x = abs(starting_x-robot_x)
        drive(delta_start_x*2.54,my_heading)
        turn(180)
        delta_start_y = abs(starting_y-robot_y)
        drive(delta_start_y*2.54,180)
        my_heading = 0
        turn(my_heading)
    else:
        drive_simple(3.5,10)
        my_heading-=90 # 0 or 180 after
        turn(my_heading)
        box_handling()
        drive_simple(-code[1]+2.54,10)
        
        goal_delta_x = goal_x-robot_x
        goal_delta_y = goal_y-robot_y

        if(goal_delta_x<0):
            my_heading=270
        else:
            my_heading = 90

        turn(my_heading)

        drive(abs(goal_delta_x)*2.54,my_heading)

        robot_x=goal_x

        if(goal_delta_y<0):
            my_heading = 180
        else:
            if(my_heading == 270):
                my_heading=360
            else:
                my_heading=0
        
        turn(my_heading)
        
        drive((abs(goal_delta_y)-12)*2.54,my_heading)

        if(goal_y == -6):
            robot_y = 6
        else:
            robot_y = goal_y-12

        nav.drop_lift()

        drive_simple(-10,10)

        nav.raise_lift()

        drive_simple(6,10)

        delta_start_y = abs(starting_y-robot_y)
        delta_start_x = abs(starting_x-robot_x)

        if(delta_start_y>12):
            my_heading = 180
            turn(my_heading)
            drive((delta_start_y-12)*2.54,my_heading)
        
        if(delta_start_x>0):
            my_heading = 270
            turn(my_heading)
            drive(delta_start_x*2.54,my_heading)
        
        my_heading = 180
        turn(my_heading)
        drive_simple(12*2.54,20)
        my_heading = 0
        turn(my_heading)





    

def subtask1(box_location):
    drive(36*2.54,0)
    turn(90)
    first_dist = box_location*6+3+6
    drive(first_dist*2.54,90)
    sleep(5)
    second_dist = 48*2-first_dist
    drive(second_dist*2.54,90)
    turn(180)
    drive(36*2.54,180)

def subtask2():
    turn(0)
    drive(12*2.54,0)
    turn(-90)
    drive(48*2*2.54,-90)
    turn(-180)
    drive(12*2.54,-180)

def subtask3and4(box_location,box_number):
    global my_heading
    my_heading = 0
    first_dist = box_location*6+3
    drive(first_dist*2.54,0)
    code = read_barcode(target_dist=3)
    if(box_number==code[0]):
        devices.speaker.speak("CORRECT BOX, THIS IS THE CORRECT BOX")
        print("RIGHT BOX")
    else:
        devices.speaker.speak("WRONG BOX, THIS IS THE WRONG BOX, THIS IS NOT A DRILL, AAAAAAAAAAAAAAAAAAAAA")
        print("WRONG BOX")
    input()
    drive_simple(3.5,10)
    turn(-90)
    box_handling()
    drive_simple(-code[1],10)
    turn(0)
    second_dist = 42-first_dist
    drive(second_dist*2.54,0)
    nav.drop_lift()
    drive_simple(-10,10)
    

    






    

def main():
    nav.initialize(devices)   
    sleep(0.5)
    left_motor.off(False)
    right_motor.off(False)
    devices.ultrasonic.distance_centimeters_continuous
    #subtask1(4)
    #input()
    #subtask2()
    #subtask3and4(2,0)
    #mainTask("A1",11,"B",0)
    #goToPosition(0,0,3,False,0,2)
    #nav.equalize_motors()
    #drive(60,0)
    #nav.calibrate_via_color_dist()
    #nav.calibrate_via_rotation()

    #devices.lift_motor.on_for_rotations(100,-1)
    




main()
