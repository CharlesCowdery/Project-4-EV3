import random
import json
import math
import copy
import time
import navigation_r as nav
import numpy
import pyximport; pyximport.install()
import navperf
import os
from multiprocessing.pool import Pool
import datetime

#index = None

def initialize(_index):
    global index
    index = _index


def pad(data,padding_size):
    my_data = data.copy()
    for i in range(padding_size): #add padding
        my_data.insert(0,0)
        my_data.append(0)
    return my_data

def apply_kernel(data,kernel,padding = True):
    if(len(kernel)%2 == 1):
        kernel_size = len(kernel)
        half_kernel_size = math.floor(kernel_size/2)
        start_index = half_kernel_size
        end_index = len(data)-half_kernel_size

        if(padding == True):
            my_data = pad(data,half_kernel_size)
            start_index = 0
            end_index = len(data)
        else:
            my_data = data

        length = end_index-start_index
        accumulator = [0]*(length)

        for i in range(length):
            for k in range(kernel_size):
                index = (start_index+i+k)-half_kernel_size
                accumulator[i]+=my_data[index]*kernel[k]
        
        return accumulator
    else:
        raise Exception("A kernel of size {} is invalid!".format(len(kernel)))


def spawnKernels(min_reach,max_reach,sibling_count,rand_scalar = 1):
    base_kernels = []
    for half_size in range(min_reach,max_reach+1):
        kernel = [0]*(1+half_size*2)
        base_kernels.append(kernel.copy())

    kernels = []
    for i in range(len(base_kernels)):
        template_kernel = base_kernels[i]
        template_length = len(template_kernel)
        for k in range(sibling_count):
            kernel = template_kernel.copy()
            for p in range(template_length):
                kernel[p] = (random.random()*2-1)*rand_scalar
            kernels.append(kernel)
                
    return kernels

def load_padded_data(index_file_name,max_padding):
    index = json.load(open(index_file_name))
    for catagory in index:
        files = index[catagory]["files"]
        data = dict()
        datasets = dict()
        for file_name in files:
            file_data = nav.load_nav_data(file_name)
            data[file_name] = file_data

            data[file_name][3].append(-9999)

            sub_set = dict()
            sub_set["left"] = numpy.asarray(file_data[1])
            sub_set["right"] = numpy.asarray(file_data[2])

            datasets[file_name] = sub_set
        
        index[catagory]["file-data"] = data
        index[catagory]["data-sets"] = datasets
    return index

def transform(v):
    return math.pow((v+1),3)

def linear_fitness(delta_x_arr,delta_y_arr,weight):
    total_abs_x = 0
    total_abs_y = 0
    for delta_x in delta_x_arr:
        total_abs_x+=transform(abs(delta_x))
    for delta_y in delta_y_arr:
        total_abs_y+=transform(abs(delta_y))
    return 1/(total_abs_y)*len(weight)
    #*len(weight) just makes fitness more readable, it doesnt affect results

fitness_function = linear_fitness
time_spend_reconstructing = 0
time_spend_kerneling = 0

def test_kernel(catagory,file_name,kernel):
    left = catagory["data-sets"][file_name]["left"]
    right = catagory["data-sets"][file_name]["right"]
    kerneled_left = numpy.convolve(left,kernel)
    kerneled_right = numpy.convolve(right,kernel)
    expected_x = catagory["file-data"][file_name][4]
    expected_y = catagory["file-data"][file_name][5]
    return [navperf.reconstruct(kerneled_left, kerneled_right, kerneled_left.size),[expected_x,expected_y]]

def get_fitness(index,kernel,return_deltas = False):
    global time_spend_reconstructing, time_spend_kerneling
    delta_x_arr = []
    delta_y_arr = []
    weights = []
    reconstruction_time = 0
    kerneling_time = 0
    for catagory_name in index:
        weight = index[catagory_name]["weight"]
        catagory = index[catagory_name]
        for file_name in catagory["data-sets"]:
            dataset = catagory["data-sets"][file_name]

            kerneling_time -= time.perf_counter()
            kerneled_left = numpy.convolve(dataset["left"],kernel)
            kerneled_right = numpy.convolve(dataset["right"],kernel)
            #kerneled_left = apply_kernel(dataset["left"],kernel,False)
            #kerneled_right = apply_kernel(dataset["right"],kernel,False)
            kerneling_time += time.perf_counter()


            angles = catagory["file-data"][file_name][3]

            expected_x = catagory["file-data"][file_name][4]
            expected_y = catagory["file-data"][file_name][5]


            reconstruction_time -= time.perf_counter()
            results = navperf.reconstruct(kerneled_left, kerneled_right, kerneled_left.size)
            reconstruction_time += time.perf_counter()

            #time_spend_reconstructing+=delta_rec

            #time_spend_kerneling+=delta_kernel
            
            delta_x_arr.append(results[0]-expected_x)
            delta_y_arr.append(results[1]-expected_y)
            weights.append(weight)
    if(not return_deltas):
        return [fitness_function(delta_x_arr,delta_y_arr,weights),reconstruction_time,kerneling_time]
    else:
        return [delta_x_arr,delta_y_arr]


def get_fitness_set(kernels,child_in_place=False):
    fitnesses = kernels.copy()
    reconstruction_time = 0
    kerneling_time = 0
    total_time = -time.perf_counter()
    fitness_time = 0
    for kernel_index in range(len(kernels)):
        if child_in_place:
            child_set = kernels[kernel_index]
            for child_index in range(len(child_set)):

                fitness_time+=-time.perf_counter()
                results = get_fitness(index,child_set[child_index])
                fitness_time+=time.perf_counter()

                fitnesses[kernel_index][child_index] = results[0]
                reconstruction_time += results[1]
                kerneling_time += results[2]
        else:  
            kernel = kernels[kernel_index]

            fitness_time+=-time.perf_counter()
            results = get_fitness(index,kernel)
            fitness_time+=time.perf_counter()

            fitnesses[kernel_index] = results[0]
            reconstruction_time += results[1]
            kerneling_time += results[2]
    total_time+=time.perf_counter()
    #return [numpy.asarray(fitnesses),reconstruction_time,kerneling_time]
    return [fitnesses,reconstruction_time,kerneling_time,total_time,fitness_time]


def spawn_children(kernel,offspring_count,magnitude):
    child_array = []
    for i in range(offspring_count):
        child_kernel = kernel.copy()
        for position in range(len(kernel)):
            child_kernel[position]+=(random.random()*2-1)*magnitude
        child_array.append(child_kernel)
    return child_array


def spawn_children_set(kernels,offspring_count,magnitude):
    children = copy.deepcopy(kernels)
    for kernel_index in range(len(kernels)):
        kernel = kernels[kernel_index]
        children[kernel_index] = spawn_children(kernel,offspring_count,magnitude)
    return children


def select_fitess_member(parent,parent_fitness,children,children_fitness):
    highest_fitness = parent_fitness
    highest_fitness_kernel = parent
    for child_index in range(len(children_fitness)):
        if(children_fitness[child_index] > highest_fitness):
            highest_fitness = children_fitness[child_index]
            highest_fitness_kernel = children[child_index]
    
    return (highest_fitness_kernel,highest_fitness)



def select_fitess_member_set(parents,parents_fitness,children,children_fitness):
    fitnesses = copy.deepcopy(parents_fitness)
    highest_fitness_kernels = copy.deepcopy(parents_fitness)
    max_fitness = 0
    best_kernel = []
    total_fitness = 0
    total_kernels = 0
    for kernel_index in range(len(parents_fitness)):
        selection_results = select_fitess_member(
                parents[kernel_index],
                parents_fitness[kernel_index],
                children[kernel_index],
                children_fitness[kernel_index]
            )
        
        highest_fitness_kernels[kernel_index] = selection_results[0]
        fitnesses[kernel_index] = selection_results[1]

        total_kernels+=1
        total_fitness+=selection_results[1]

        if(selection_results[1]>max_fitness):
            max_fitness = selection_results[1]
            best_kernel = selection_results[0]
    
    return (highest_fitness_kernels,fitnesses,max_fitness,best_kernel,total_fitness/total_kernels)


def sort_kernels(kernels,fitnesses):
    def sorter(e):
            return e[1]
        
    sortable_array = []
    for i in range(len(kernels)):
        sortable_array.append([kernels[i],fitnesses[i]])
    
    sortable_array.sort(key = sorter, reverse = True)

    kernel_array = []
    fitness_array = []

    for i in range(len(sortable_array)):
        kernel_array.append(sortable_array[i][0])
        fitness_array.append(sortable_array[i][1])

    return [kernel_array,fitness_array,sortable_array]


def prune(kernels,fitnesses,percent):
    
    sortable_array = sort_kernels(kernels,fitnesses)[2]

    size = len(sortable_array)
    clipping_size = math.floor(size*(1-percent))
    final_kernels = []
    final_fitnesses = []
    for i in range(size):
        if(i<clipping_size):
            final_kernels.append(sortable_array[i][0].copy())
            final_fitnesses.append(sortable_array[i][1])
        else:
            final_kernels.append(sortable_array[i%clipping_size][0].copy())
            final_fitnesses.append(sortable_array[i%clipping_size][1])
    print()
    print("   -pruned {:.1f}%".format(percent*100))

    return [final_kernels,final_fitnesses]