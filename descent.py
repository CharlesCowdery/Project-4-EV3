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

def linear_fitness(delta_x_arr,delta_y_arr,weight):
    total_abs_x = 0
    total_abs_y = 0
    for delta_x in delta_x_arr:
        total_abs_x+=math.exp(math.sqrt(abs(delta_x)**1.5))-1
    for delta_y in delta_y_arr:
        total_abs_y+=math.exp(math.sqrt(abs(delta_y)**1.5))-1
    return 1/(total_abs_x)*len(weight)
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

def get_fitness(index,kernel):
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
    return [fitness_function(delta_x_arr,delta_y_arr,weights),reconstruction_time,kerneling_time]


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


def get_fitness_threaded(index,kernels,thread_count,pool):
    chunk_size = math.floor(len(kernels)/thread_count)

    reconstruction_time = 0
    kerneling_time = 0

    chunks = []

    workers = 0

    final = []

    array_time = -time.perf_counter()
    for i in range(1,thread_count+1):
        start_index = ((i-1)*chunk_size)
        end_index = (i*chunk_size)
        chunk = kernels[start_index:end_index]
        chunks.append((chunk,True))
    array_time += time.perf_counter()
    
    deploying_workers_time = -time.perf_counter()
    workers =(pool.starmap_async(get_fitness_set, chunks, chunksize=1))
    deploying_workers_time += time.perf_counter()

    worker_time = 0
    fitness_time = 0

    worker_time += -time.perf_counter()
    results = workers.get()
    worker_time += time.perf_counter()
    

    #print(results)
    total_time = 0
    for i in range(len(results)):
        result = results[i]
        #print(result)
        array_time += -time.perf_counter()
        final+=result[0]
        reconstruction_time+=result[1]/thread_count
        kerneling_time += result[2]/thread_count
        total_time+=result[3]/thread_count
        fitness_time += result[4]/thread_count
        array_time += time.perf_counter()

    return [final,
            reconstruction_time,
            kerneling_time,
            array_time,
            deploying_workers_time+(worker_time-total_time),
            total_time,
            fitness_time]



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
    print("   -pruned {}%".format(percent*100))

    return [final_kernels,final_fitnesses]


def generate_report(best_kernels,best_fitnesses,epochs,initial_magnitude,generations,
                    index_file_name,index,max_reach,average_fitness_pre,
                    average_fitness_post,median_fitness_pre,median_fitness_post,
                    offspring_count,log_file_name="descent-latest.log"):
    log = dict()
    metadata = dict()
    metadata["index-file"] = index_file_name
    metadata["epoch-count"] = epochs
    metadata["max-reach"] = max_reach
    metadata["generations"] = generations
    metadata["offspring-count"] = offspring_count
    metadata["initial-magnitude"] = initial_magnitude
    metadata["best-kernel"] = best_kernels[len(best_kernels)-1]
    metadata["best-fitness"] = best_fitnesses[len(best_kernels)-1]
    metadata["description"] = "no description"

    log["metadata"] = metadata


    epochs_data = dict()

    for epoch in range(0,epochs):
        epoch_data = dict()
        best_kernel = best_kernels[epoch]
        epoch_data["best-kernel"] = best_kernel
        epoch_data["best-fitness"] = best_fitnesses[epoch]
        epoch_data["results"] = dict()
        if(type(generations) == list):
            epoch_data["generations"] = generations[epoch]
        else:
            epoch_data["generations"] = generations
        epoch_data["magnitude"] = initial_magnitude/(epoch+1)
        epoch_data["average-fitness-preop"] = average_fitness_pre[epoch]
        epoch_data["average-fitness-postop"] = average_fitness_post[epoch]
        epoch_data["median-fitness-preop"] = median_fitness_pre[epoch]
        epoch_data["median-fitness-postop"] = median_fitness_post[epoch]

        for catagory_name in index:
            catagory = index[catagory_name]
            epoch_data[catagory_name] = dict()
            for file_name in catagory["files"]:
                result_data = dict()
                results = test_kernel(catagory,file_name,best_kernel)
                experimental = dict()
                predicted = dict()
                experimental["x"] = results[1][0]
                experimental["y"] = results[1][1]
                predicted["x"] = results[0][0]
                predicted["y"] = results[0][1]
                result_data["experimental"] = experimental
                result_data["predicted"] = predicted
                epoch_data[catagory_name][file_name] = result_data

        epochs_data["epoch-{}".format(epoch+1)] = epoch_data
    
    log["epochs"] = epochs_data

    file = open(log_file_name,"w")
    file.write(json.dumps(log,indent=4))
    file.close()

def init_worker(data):
        global index
        index = data

def descend(index_file_name,max_reach,min_reach,
            generations=100,    #number of iterations each epoch
            epochs=3,           #this is how many "phases" there are. per epoch it runs the generations, and then halves the magnitude
            offspring_count=1,        #number of offshoot kernels tested every generation
            initial_magnitude=1,#starting max magnitude of modifications. Halved every generation
            sibling_multiplier=10,#number of kernels generated per size
            operators = None,
            threads=os.cpu_count()
            ):
    global index
    kernels = spawnKernels(min_reach,max_reach,sibling_multiplier*threads,1)
    index = load_padded_data(index_file_name,max_reach)
    magnitude = initial_magnitude
    

    
    print("initializing pool")
    
    pool = Pool(processes=threads,initializer=init_worker,initargs=(index,))
    print("Pool initialized")

    print("Total kernel load: "+str(len(kernels)*offspring_count))
    console_width = (os.get_terminal_size())[0]

    cols = console_width

    best_kernels=[]
    best_fitnesses=[]
    average_fitness_preop = []
    average_fitness_postop = []
    median_fitness_preop = []
    median_fitness_postop = []

    eval_time = 1
    array_time = 0
    time_spend_reconstructing = 0
    time_spend_kerneling = 0
    worker_deploy_time = 0
    worker_time = 1
    fitness_time = 1
    for epoch in range(epochs):
        print("epoch {} (magnitude:{}):".format(epoch+1,magnitude))
        prev_fitness = get_fitness_set(kernels)[0]
        max_fitness = 0
        avg_fitness = 0
        gen_time = 1

        best_kernel = 0

        if type(generations) == list:
            generation_length = generations[epoch]
        else:
            generation_length = generations

        for generation in range(generation_length):
            start = time.perf_counter()
            verbose = True
            if(verbose):
                print("\r -> running generation "+str(generation+1))
                print(" "*cols,end="\r")
                print("   -> best fitness of last gen: {:.4f}".format(max_fitness))
                print(" "*cols,end="\r")
                best_kernel_text = "   -> best kernel last gen: "+str(best_kernel)
                print(best_kernel_text)
                print(" "*cols,end="\r")
                print("   -> avg fitness: {:.6f}".format(avg_fitness))
                print(" "*cols,end="\r")
                print("   -> total generation time: {:.6f}s".format(gen_time))
                print(" "*cols,end="\r")
                print("      -> evaluation time: {:.6f}s ({:.1f}%)".format(eval_time,eval_time/gen_time*100))
                print(" "*cols,end="\r")             
                print("         -> array manipulation: {:.1f}%".format(array_time/eval_time*100))
                print(" "*cols,end="\r")
                print("         -> worker deploy: {:.1f}%".format(worker_deploy_time/eval_time*100))
                print(" "*cols,end="\r")
                print("         -> waiting on workers: {:.1f}%".format(worker_time/eval_time*100))
                print(" "*cols,end="\r")
                print("            -> calculating fitness: {:.1f}%".format(fitness_time/worker_time*100))
                print(" "*cols,end="\r")
                print("                -> reconstruction: {:.1f}%".format(time_spend_reconstructing/fitness_time*100))
                print(" "*cols,end="\r")
                print("                -> kerneling: {:.1f}%".format(time_spend_kerneling/fitness_time*100))
                if(generation < generation_length-1):
                    print("\033[F"*11,end="")
                    for i in range(math.ceil(len(best_kernel_text)/console_width)):
                        print("\033[F",end="")
                    1+1
            time_spend_reconstructing = 0
            time_spend_kerneling = 0

            
            children = spawn_children_set(kernels,offspring_count,magnitude)
            #print(children)
            eval_time = -time.perf_counter()
            fitness_results = get_fitness_threaded(index,children,threads,pool)
            eval_time += time.perf_counter()

            child_fitness = fitness_results[0]
            time_spend_reconstructing = fitness_results[1]
            time_spend_kerneling = fitness_results[2]
            array_time = fitness_results[3]
            worker_deploy_time = fitness_results[4]
            worker_time = fitness_results[5]
            fitness_time = fitness_results[6]
            #child_fitness = get_fitness_set(index,children,True)[0]

            selection_results = select_fitess_member_set(kernels,prev_fitness,children,child_fitness)

            kernels = selection_results[0]
            prev_fitness = selection_results[1]

            max_fitness = selection_results[2]
            best_kernel = selection_results[3]
            avg_fitness =selection_results[4]

            end = time.perf_counter()
            gen_time = end-start

            #if(generation>2):
            #    pool.close()
            #    exit()
        
        magnitude = magnitude/2

        best_kernels.append(best_kernel)
        best_fitnesses.append(max_fitness)
        average_fitness_preop.append(avg_fitness)

        sorted = sort_kernels(kernels,prev_fitness)
        kernels = sorted[0]
        prev_fitness = sorted[1]
        median_fitness_preop.append(prev_fitness[math.floor(len(kernels)/2)])

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

        sorted = sort_kernels(kernels,prev_fitness)
        kernels = sorted[0]
        prev_fitness = sorted[1]
        median_fitness_postop.append(prev_fitness[math.floor(len(kernels)/2)])

        total_postop_fitness = 0
        for fitness in prev_fitness:
            total_postop_fitness+=fitness
        
        average_fitness_postop.append(total_postop_fitness/len(prev_fitness))

        print()

    print("evolution done. Closing worker pool.")
    pool.close()

    print("Exporting report to descent-latest.log")
    generate_report(
        index_file_name=index_file_name,
        index=index,
        epochs=epochs,
        initial_magnitude=initial_magnitude,
        offspring_count=offspring_count,
        median_fitness_post=median_fitness_postop,
        median_fitness_pre=median_fitness_preop,
        average_fitness_pre=average_fitness_preop,
        average_fitness_post=average_fitness_postop,
        max_reach=max_reach,
        best_kernels=best_kernels,
        generations=generations,
        best_fitnesses=best_fitnesses,
        log_file_name="descent-log.json"
        )
        



if __name__ == "__main__":
    print()
    descend("other/indexes/descent-data.json",
            max_reach=6,
            min_reach=4,
            generations=[200,200,200,200,100,100,100,100],
            epochs=7,
            offspring_count=3,
            sibling_multiplier=100,
            operators=[[prune,[0.5]],[prune,[0.15]],[prune,[0.20]],[prune,[0.30]],[prune,[0.75]],[prune,[0.75]],None]
            )
    #data = (load_padded_data("other/indexes/descent-data.json",3))
    #angles = data["60"]["file-data"]['other/nav-logs/cm-60-woodpanel-maxbattery.json'][3]
    #left = data["60"]["data-sets"]['other/nav-logs/cm-60-woodpanel-maxbattery.json']["left"]
    #right = data["60"]["data-sets"]['other/nav-logs/cm-60-woodpanel-maxbattery.json']["right"]

    

    #print(nav.reconstruct(angles,left,right,False))

