from time import sleep
from datetime import datetime
import math
import json
import os
import constants as con
from descent import apply_kernel


#This file contains functions pertaining to sensor calibration
#navigation estimation, and log import and exporting

gyro = 0
left_motor = 0
right_motor = 0
move_tank = 0

def initialize(devices):  #We get take a devices instance so the modules can share a device set
    global gyro,left_motor,right_motor,move_tank
    gyro = devices.gyro
    left_motor = devices.left_motor
    right_motor = devices.right_motor
    move_tank = devices.move_tank

    print("Calibrating!")
    gyro.calibrate()
    gyro.reset()
    print("Done!")

    gyro.mode = gyro.MODE_GYRO_ANG




def calibrate():
    starting_angle = gyro.angle
    starting_left = left_motor.position/360
    starting_right = right_motor.position/360
    left_motor.on(10)
    right_motor.on(-10)
    while(gyro.angle<starting_angle+720-2):
        1+1
    left_motor.off(True)
    right_motor.off(True)
    sleep(1)
    delta_left = left_motor.position/360-starting_left
    delta_right = right_motor.position/360-starting_right
    print(delta_left)
    print(delta_right)
    print(gyro.angle)
    length = gyro.angle/360*math.pi*con.cross_section_length
    average_rotation = (abs(delta_left)+abs(delta_right))/2
    print(length/average_rotation)
    
def angle_from_dist_square(l,r): #this predicts angle based off of movement of the wheels.
    short = l
    long = r
    modifier = 1
    if(l < 0 or r < 0):
        raise Exception("Negative input to angle_from_dist! this behavior is undefined!")
        return 0
    if(l == r):
        return 0
    if(short>long):
        short = r
        long = l
        modifier = -1

    k = con.cross_section_length            #for brevity                                     
    midline = math.sqrt(short**2+k**2)                         
    product1 = math.asin(k/midline)     #gets top angle of the right angle triangle
    product2 = math.acos(-(long**2-midline**2-k**2)/(2*midline*k))   #gets the complementary angle for the other triangle
    total_change = (product1+product2)*180/math.pi-90
    return total_change*modifier

    #sorry the math for this is a bit messy
    #this is operating off a model where it forms a box with both the top and bottom length being the cross section length, and the sides the distances.
    #It assumes the short side forms a right angle, and the left side is acute on the bottom right angle

    #shoot me a text if you want a diagram


    # -Sel
    

def model_turn_circle(l,r): #this wont error out when given a negative number, but the behavior is unaccounted for, and should be avoided
    inside = l*con.distance_scalar
    outside = r*con.distance_scalar
    delta = outside-inside
    length = inside

    if(l == 0 or r == 0): #todo, make this actually useful. Not super high priority though, since input kernels means zeros are rare
        return [0,0,0,0]
    if(delta == 0):
        return [0,0,0,length]#l
    

    #this assumes the robots tires essentially form two concentric circles when driving at different rates
    #this models that, and find the displacement.
    #I'd explain the math, but its all algebra'd to hell, so just ask me to send a photo if you need it
    internal_radius = inside*con.cross_section_length/delta
    theta = delta/con.cross_section_length
    mid_radius = con.cross_section_length/2+internal_radius
    displacement_x  = math.cos(theta)*mid_radius-mid_radius
    displacement_y = math.sin(theta)*mid_radius
    

    if(displacement_x != 0):
        raw_theta = math.atan(displacement_y/displacement_x)
    else:
        raw_theta = math.pi/2

    if raw_theta>0:
        displacement_angle = raw_theta-math.pi/2
    else:
        displacement_angle = (math.pi/2+raw_theta)

    

    length = math.sqrt(displacement_x*displacement_x + displacement_y*displacement_y)


    return [displacement_x,displacement_y,displacement_angle,length]


def reconstruct(angles,l_rotations,r_rotations,verbose=False,file_name="NEWREC.log"):
    if(verbose):print("reconstructing route:")
    deltas_l = []
    deltas_r = []
    i = 1

    if(verbose):print(" -> Getting rotation deltas...")
    while(angles[i]!=-9999):
        deltas_l.append(l_rotations[i]-l_rotations[i-1])
        deltas_r.append(r_rotations[i]-r_rotations[i-1])
        i+=1
    
    x = 0
    y = 0

    angle = -angles[0]/180*math.pi + math.pi/2       

    if(verbose):print(" -> calculating position changes...")
    for i in range(len(deltas_l)):
        prediction = model_turn_circle(deltas_l[i],deltas_r[i])
        angle += prediction[2]

        x+=prediction[3]*math.cos(angle)
        y+=prediction[3]*math.sin(angle)

        if(verbose):
            file_string+="'delta_left':{0:9.6f}, 'delta_right':{1:9.6f}, 'delta_angle':{2:7.2f} 'gyro':{3:5.0f}, 'accumulated_angle':{4:7.2f}, 'predict_x':{5:5.4f}, 'predict_y':{6:5.1f}\n".format(
                    deltas_l[i],deltas_r[i],prediction[2],-angles[i],angle,x,y)
        #avg_rotation = (deltas_l[i]+deltas_r[i])/2
        #x+=avg_rotation*math.cos((angle+90)*math.pi/180)*distance_scalar
        #y+=avg_rotation*math.sin((angle+90)*math.pi/180)*distance_scalar
    
    if(verbose):
        print(" -> route reconstruction done! Final change: X = {0:.3f} ; Y = {1:.3f}".format(x,y))
        print(" -> writing log file...")
        log_file = open(os.path.join("reconstructions",file_name),"w")
        log_file.write(file_string)
        log_file.close()
        print(" -> file written and saved!")
        print()
    
    return (x,y)

def load_nav_data(file_name,endians = False):
    file = open(os.path.join(file_name),"r")
    nav_obj = json.load(file)

    metadata = nav_obj["metadata"]
    path_data = nav_obj["path-data"]

    angles = []
    left_rotations = []
    right_rotations = []

    for i in range(len(path_data)):
        data_block = path_data[i]
        angles.append(data_block["angle"])
        left_rotations.append(data_block["left"])
        right_rotations.append(data_block["right"])
    
    if endians:
        angles.append(-9999)
        left_rotations.append(-9999)
        right_rotations.append(-9999)
    
    return (metadata,left_rotations,right_rotations,angles,metadata["final-x"],metadata["final-y"])

def reconstruct_from_file(file_name,kernel = False):
    
    nav_data = load_nav_data(file_name,False)
    metadata = nav_data[0]
    left_rotations = nav_data[1]
    right_rotations = nav_data[2]
    angles = nav_data[3]

    if(kernel!=False):                      
        left_rotations = apply_kernel(left_rotations,kernel)
        right_rotations = apply_kernel(right_rotations,kernel)

    angles.append(-9999)
    left_rotations.append(-9999)
    right_rotations.append(-9999)

    new_file_name = "REC-"+file_name.split(".")[0]+".log"
    reconstruct(angles, left_rotations, right_rotations, False, new_file_name)


def export_movement(name,angles,left_rotations,right_rotations,size,dist,target_angle,start_time,end_time):
    print("Beginning movement export.")

    print(" -> writing metadata...")
    now = datetime.now()
    file_string = "{"
    metadata = '\t"metadata": {'+"""
    \t\t"distance": {0:.2f},
    \t\t"target-angle": {1:.2f},
    \t\t"scalar": {2:.3f},
    \t\t"file-time": "{3}",
    \t\t"start-time": {4},
    \t\t"end-time": {5},
    \t\t"nominal-polling-rate": {6},
    \t\t"final-x": null,
    \t\t"final-y": null
    """.format(
            dist,target_angle,con.distance_scalar,now,start_time,end_time,size/(end_time-start_time)
        )+"},\n"
    
    print(" -> writing movement logs...")
    log_contents = '\t"path-data": [\n'
    for i in range(size):
        data_string = "\t\t{"
        data_string += '"left": {:.10f}, "right": {:.10f}, "angle": {:5.0f}'.format(left_rotations[i],right_rotations[i],angles[i])
        data_string += "}"
        if(i != size-1):
            data_string+=",\n"
        else:
            data_string+="\n"
        log_contents+=data_string
    log_contents+="\t]\n"
    file_string+=metadata+log_contents+"}"

    print(" -> file construction done!")
    print(" -> exporting into file: "+name)

    file = open(name,"w")
    file.write(file_string)
    file.close()

    print(" -> Movement export done!")

if __name__ == "__main__":
    #[-1,3,1]/4.88 works really well for some fucking reason
    reconstruct_from_file("nav-logs/cm-60-woodpanel-maxbattery.json",[-1,3,1])