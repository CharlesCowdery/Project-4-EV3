import random
import json
import math
import copy
import navigation as nav

def pad(data,padding_size):
    my_data = data.copy()
    for i in range(padding_size): #add padding
        my_data.insert(0,0)
        my_data.append(0)
    return my_data

def apply_kernel(data,kernel,pad = True):
    if(len(kernel)%2 == 1):
        kernel_size = len(kernel)
        half_kernel_size = math.floor(kernel_size/2)
        start_index = half_kernel_size
        end_index = len(data)-half_kernel_size

        if(pad == True):
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
    for half_size in range(1,max_reach):
        kernel = [0]*(1+half_size*2)
        base_kernels.append(kernel.copy())

    kernels = []
    for i in range(len(base_kernels)):
        sub_kernel = []
        template_kernel = base_kernels[i]
        template_length = len(template_kernel)
        for k in range(sibling_count):
            kernel = template_kernel.copy()
            for p in range(template_length):
                kernel[p] = (random.random()*2-1)*rand_scalar
            
            sub_kernel.append(kernel)    
        kernels.append(sub_kernel.copy())
    
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

            dataset = []
            for padding_size in range(1,max_padding+1):
                sub_set = dict()
                sub_set["padding-size"] = padding_size
                sub_set["left"] = pad(file_data[1],padding_size)
                sub_set["right"] = pad(file_data[2],padding_size)
                dataset.append(sub_set)

            datasets[file_name] = dataset
        
        index[catagory]["file-data"] = data
        index[catagory]["data-sets"] = datasets
    return index

def linear_fitness(d_x,d_y):
    return 1/(abs(d_x)+abs(d_y))

fitness_function = linear_fitness

def get_fitness(index,kernel):
    dataset_index = math.floor(len(kernel)/2)-1
    total_fitness = 0
    for catagory_name in index:
        weight = index[catagory_name]["weight"]
        catagory = index[catagory_name]
        for file_name in catagory["data-sets"]:
            dataset = catagory["data-sets"][file_name][dataset_index]

            kerneled_left = apply_kernel(dataset["left"],kernel,False)
            kerneled_right = apply_kernel(dataset["right"],kernel,False)

            kerneled_left.append(-9999)
            kerneled_right.append(-9999)

            angles = catagory["file-data"][file_name][3]

            expected_x = catagory["file-data"][file_name][4]
            expected_y = catagory["file-data"][file_name][5]

            results = nav.reconstruct(angles, kerneled_left, kerneled_right)

            total_fitness += fitness_function(results[0]-expected_x,results[1]-expected_y)*weight
    return total_fitness


def get_fitness_set(index,kernels):
    fitnesses = copy.deepcopy(kernels)
    for padding_set_index in range(len(kernels)):
        padding_set = kernels[padding_set_index]
        for kernel_index in range(len(padding_set)):
            kernel = padding_set[kernel_index]
            fitnesses[padding_set_index][kernel_index] = get_fitness(index,kernel)
    return fitnesses



def descend(index_file_name,max_reach,
            generations=100,    #number of iterations each epoch
            epochs=3,           #this is how many "phases" there are. per epoch it runs the generations, and then halves the magnitude
                                #and after the safe number of epochs, begins to prune away the worst kernels
            pruning_rate=0.5,   #this is the percent of the bottom kernels that will be pruned away and replaced
            offspring=1,        #number of offshoot kernels tested every generation
            initial_magnitude=1,#starting max magnitude of modifications. Halved every generation
            starting_siblings=10,#number of kernels generated per size
            safe_epochs=1       #number of safe epochs
            ):
    kernels = spawnKernels(max_reach,starting_siblings,1)
    index = load_padded_data(index_file_name,max_reach)
    magnitude = initial_magnitude
    for epoch in range(epochs):
        print("epoch {}:".format(epoch+1))
        prev_fitness = get_fitness_set(index,kernels)
        for generation in range(generations):
            print("\r -> running generation "+str(generation+1),end="")

        print()
        



if __name__ == "__main__":
    descend("indexes/descent-data.json",10)
    #print(load_padded_data("indexes/descent-data.json",3))
