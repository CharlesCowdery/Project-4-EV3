import random
import json
import math
import copy
import time
import navigation as nav
import numpy
import pyximport; pyximport.install()
import navperf

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


def spawnKernels(max_reach,sibling_count,rand_scalar = 1):
    base_kernels = []
    for half_size in range(1,max_reach+1):
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

def linear_fitness(d_x,d_y):
    return 1/(abs(d_x))#+abs(d_y))

fitness_function = linear_fitness
time_spend_reconstructing = 0
time_spend_kerneling = 0

def get_fitness(index,kernel):
    global time_spend_reconstructing, time_spend_kerneling
    dataset_index = math.floor(len(kernel)/2)-1
    total_fitness = 0
    for catagory_name in index:
        weight = index[catagory_name]["weight"]
        catagory = index[catagory_name]
        for file_name in catagory["data-sets"]:
            dataset = catagory["data-sets"][file_name]

            start_kernel = time.perf_counter()
            kerneled_left = numpy.convolve(dataset["left"],kernel)
            kerneled_right = numpy.convolve(dataset["right"],kernel)
            #kerneled_left = apply_kernel(dataset["left"],kernel,False)
            #kerneled_right = apply_kernel(dataset["right"],kernel,False)
            end_kernel = time.perf_counter()


            angles = catagory["file-data"][file_name][3]

            expected_x = catagory["file-data"][file_name][4]
            expected_y = catagory["file-data"][file_name][5]

            start_rec = time.perf_counter()
            results = navperf.reconstruct(kerneled_left, kerneled_right, kerneled_left.size)
            end_rec = time.perf_counter()

            delta_rec = end_rec-start_rec
            time_spend_reconstructing+=delta_rec

            delta_kernel = end_kernel-start_kernel
            time_spend_kerneling+=delta_kernel

            total_fitness += fitness_function(results[0]-expected_x,results[1]-expected_y)*weight
    return total_fitness


def get_fitness_set(index,kernels,child_in_place=False):
    fitnesses = copy.deepcopy(kernels)
    for kernel_index in range(len(kernels)):
        if child_in_place:
            child_set = kernels[kernel_index]
            for child_index in range(len(child_set)):
                fitnesses[kernel_index][child_index] = get_fitness(index,child_set[child_index])
        else:  
            kernel = kernels[kernel_index]
            fitnesses[kernel_index] = get_fitness(index,kernel)
    return fitnesses

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
            
def prune(kernels,fitnesses,percent):
    def sorter(e):
        return e[1]
    
    sortable_array = []
    for i in range(len(kernels)):
        sortable_array.append([kernels[i],fitnesses[i]])
    
    sortable_array.sort(key = sorter, reverse = True)
    
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
    print("   -pruned {}%".format(percent*100))

    return [final_kernels,final_fitnesses]






def descend(index_file_name,max_reach,
            generations=100,    #number of iterations each epoch
            epochs=3,           #this is how many "phases" there are. per epoch it runs the generations, and then halves the magnitude
            offspring_count=1,        #number of offshoot kernels tested every generation
            initial_magnitude=1,#starting max magnitude of modifications. Halved every generation
            starting_siblings=10,#number of kernels generated per size
            operators = None
            ):
    kernels = spawnKernels(max_reach,starting_siblings,1)
    index = load_padded_data(index_file_name,max_reach)
    magnitude = initial_magnitude

    cols = 160

    for epoch in range(epochs):
        global time_spend_reconstructing, time_spend_kerneling
        print("epoch {} (magnitude:{}):".format(epoch+1,magnitude))
        prev_fitness = get_fitness_set(index,kernels)
        max_fitness = 0
        best_kernel = []
        avg_fitness = 0
        gen_time = 1
        if type(generations) == list:
            generation_length = generations[epoch]
        else:
            generation_length = generations
        for generation in range(generation_length):
            start = time.perf_counter()
            print("\r -> running generation "+str(generation+1))
            print(" "*cols,end="\r")
            print("   -> best fitness of last gen: {:.4f}".format(max_fitness))
            print(" "*cols,end="\r")
            print("   -> best kernel last gen: "+str(best_kernel))
            print(" "*cols,end="\r")
            print("   -> avg fitness: {:.6f}".format(avg_fitness))
            print(" "*cols,end="\r")
            print("   -> gen time: {:.6f}s".format(gen_time))
            print(" "*cols,end="\r")
            print("      -> percent time reconstructing: {:.1f}%".format(time_spend_reconstructing/gen_time*100))
            print(" "*cols,end="\r")
            print("      -> percent time kerneling: {:.1f}%".format(time_spend_kerneling/gen_time*100))

            time_spend_reconstructing = 0
            time_spend_kerneling = 0

            if(generation < generation_length-1):
                print("\033[F\033[F\033[F\033[F\033[F\033[F\033[F",end="")
            children = spawn_children_set(kernels,offspring_count,magnitude)
            #print(children)
            child_fitness = get_fitness_set(index,children,True)

            selection_results = select_fitess_member_set(kernels,prev_fitness,children,child_fitness)

            kernels = selection_results[0]
            prev_fitness = selection_results[1]

            max_fitness = selection_results[2]
            best_kernel = selection_results[3]
            avg_fitness =selection_results[4]

            end = time.perf_counter()
            gen_time = end-start
        magnitude = magnitude/2
        if operators!=None:
            current_operator = operators[epoch]
            if current_operator != None:
                if(type(current_operator) == list):
                    args = current_operator[1]
                    results = current_operator[0](kernels,prev_fitness, *args)
                    kernels = results[0]
                    prev_fitness = results[1]
                else:
                    results = current_operator(kernels,prev_fitness)
                    kernels = results[0]
                    prev_fitness = results[1]

        print()
        



if __name__ == "__main__":
    print()
    descend("other/indexes/descent-data.json",
            max_reach=3,
            generations=[100,100,100,100,100,100],
            epochs=6,
            offspring_count=3,
            starting_siblings=500,
            operators=[[prune,[0.5]],[prune,[0.15]],[prune,[0.20]],[prune,[0.30]],[prune,[0.75]],None]
            )
    data = (load_padded_data("other/indexes/descent-data.json",3))
    angles = data["60"]["file-data"]['other/nav-logs/cm-60-woodpanel-maxbattery.json'][3]
    left = data["60"]["data-sets"]['other/nav-logs/cm-60-woodpanel-maxbattery.json']["left"]
    right = data["60"]["data-sets"]['other/nav-logs/cm-60-woodpanel-maxbattery.json']["right"]

    

    #print(nav.reconstruct(angles,left,right,False))

