task_1 = {
    "dist":[12,36,60,84],
    "error_x" : [-0.2,0.2,0.35,1.25],
    "error_y" : [-0.55,0.55,-1.15,-0.55]
}

task_2 = {
    "dist":[12,24,48,96],
    "error_x" : [0,-0.05,-0.5,2.15],
    "error_y" : [0.05,-0.1,0.6,-1.3]
}

def predict_1(distance):
    if(distance < 0):
        return (-999,-999)
    if(distance == 0):
        return (0,0)
    if(distance <= 12):
        scalar = distance/12
        return(task_1["error_x"][0]*scalar,task_1["error_y"][0]*scalar)
    if(distance <= 36):
        diff = 36-distance
        scalar_1 = diff/24
        scalar_2 = 1-scalar_1
        x1 = task_1["error_x"][0]*scalar_1
        y1 = task_1["error_y"][0]*scalar_1
        x2 = task_1["error_x"][1]*scalar_2
        y2 = task_1["error_y"][1]*scalar_2
        return(x1+x2,y1+y2)
    if(distance <= 60):
        diff = 60-distance
        scalar_1 = diff/24
        scalar_2 = 1-scalar_1
        x1 = task_1["error_x"][1]*scalar_1
        y1 = task_1["error_y"][1]*scalar_1
        x2 = task_1["error_x"][2]*scalar_2
        y2 = task_1["error_y"][2]*scalar_2
        return(x1+x2,y1+y2)
    if(distance <= 84):
        diff = 84-distance
        scalar_1 = diff/24
        scalar_2 = 1-scalar_1
        x1 = task_1["error_x"][2]*scalar_1
        y1 = task_1["error_y"][2]*scalar_1
        x2 = task_1["error_x"][3]*scalar_2
        y2 = task_1["error_y"][3]*scalar_2
        return(x1+x2,y1+y2)
    scalar = distance/84
    return(task_1["error_x"][3]*scalar,task_1["error_y"][3]*scalar)

def predict_2(distance):
    if(distance < 0):
        return (-999,-999)
    if(distance == 0):
        return (0,0)
    if(distance <= 12):
        scalar = distance/12
        return(task_2["error_x"][0]*scalar,task_2["error_y"][0]*scalar)
    if(distance <= 24):
        diff = 24-distance
        scalar_1 = diff/12
        scalar_2 = 1-scalar_1
        x1 = task_1["error_x"][0]*scalar_1
        y1 = task_1["error_y"][0]*scalar_1
        x2 = task_1["error_x"][1]*scalar_2
        y2 = task_1["error_y"][1]*scalar_2
        return(x1+x2,y1+y2)
    if(distance <= 48):
        diff = 48-distance
        scalar_1 = diff/24
        scalar_2 = 1-scalar_1
        x1 = task_2["error_x"][1]*scalar_1
        y1 = task_2["error_y"][1]*scalar_1
        x2 = task_2["error_x"][2]*scalar_2
        y2 = task_2["error_y"][2]*scalar_2
        return(x1+x2,y1+y2)
    if(distance <= 96):
        diff = 96-distance
        scalar_1 = diff/48
        scalar_2 = 1-scalar_1
        x1 = task_2["error_x"][2]*scalar_1
        y1 = task_2["error_y"][2]*scalar_1
        x2 = task_2["error_x"][3]*scalar_2
        y2 = task_2["error_y"][3]*scalar_2
        return(x1+x2,y1+y2)
    scalar = distance/96
    return(task_2["error_x"][3]*scalar,task_2["error_y"][3]*scalar)


"""
#predicts for task 1 and task 2 based of an entered distance
while(True):
    value = input("Enter distance to predict for (in inches), or exit to leave: ")
    if(value  == "exit"):
        exit()
    try:
        dist = float(value)
        print("predicting distance: "+str(dist))
        print("\tresults for task type 1: "+str(predict_1(dist)))
        print("\tresults for task type 2: "+str(predict_2(dist)))
        print()
    except:
        print("I dont understand. Please input a valid value")
        
"""

#predicts using task 1 data for an x y movement assuming perfect turning

while(True):
    x = float(input("Enter x in inches to predict for: "))
    y = float(input("Enter y in inches to predict for: "))
    values_x = predict_1(x)
    values_y = predict_1(y)
    final = (values_x[1]+values_y[0],values_x[0]+values_y[1])
    print("Your predicted error in the x direction is {0:.2f} inches\nYour predicted error in the y direction is {1:.2f} inches".format(final[0],final[1]))
    print()
