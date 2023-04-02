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
import cuda_tools

import descent_utils as des_utils
from descent_utils import prune




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
    workers =(pool.starmap_async(des_utils.get_fitness_set, chunks, chunksize=1))
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
                results = des_utils.test_kernel(catagory,file_name,best_kernel)
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

def flatten_children(children,max_size):
    new_children = []
    for child_set in children:
        for child in child_set:
            arr = []
            child_size = len(child)
            padding_size = math.floor((max_size-child_size)/2)
            arr.append(child_size)
            arr+=[0]*padding_size
            arr+=child
            arr+=[0]*padding_size
            new_children+=(arr)
    return new_children

def deflatten_children(flat_fitness,child_group_size,group_count):
    position = 0
    arr = []
    for group_index in range(group_count):
        group_arr = []
        for i in range(child_group_size):
            group_arr.append(flat_fitness[position])
            position+=1
        arr.append(group_arr)
    return arr


def descend(index_file_name,max_reach,min_reach,
            generations=100,    #number of iterations each epoch
            epochs=3,           #this is how many "phases" there are. per epoch it runs the generations, and then halves the magnitude
            offspring_count=1,        #number of offshoot kernels tested every generation
            initial_magnitude=1,#starting max magnitude of modifications. Halved every generation
            sibling_multiplier=10,#number of kernels generated per size
            operators = None,
            threads=os.cpu_count(),
            initial_kernels = [],
            log_file_name = "descent-latest.log",
            kernel_file_name = "kernels.json"
            ):
    global index

    kernel_count = (max_reach-min_reach+1)*sibling_multiplier*threads*offspring_count
    max_size = max_reach*2+1

    print("Spawning kernels: ")
    if(len(initial_kernels)<kernel_count):
        kernel_delta = kernel_count-len(initial_kernels)
        kernels = des_utils.spawnKernels(min_reach,max_reach,sibling_multiplier*threads,1)[0:kernel_delta]+initial_kernels
    else:
        kernels = initial_kernels
    print("Done! Total kernel load: "+str(len(kernels)*offspring_count))


    index = des_utils.load_padded_data(index_file_name,max_reach)
    des_utils.initialize(_index=index)

    magnitude = initial_magnitude
    
    
    #pool = Pool(processes=threads,initializer=des_utils.initialize,initargs=(index,))


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

    best_delta_y = 0
    worst_delta_y = 0
    best_delta_x = 0
    worst_delta_x = 0

    thread_multiplier = sibling_multiplier*(max_reach-min_reach+1)*offspring_count
    cuda_tools.initialize(index,max_size,kernel_count)

    for epoch in range(epochs):
        print("epoch {} (magnitude:{}):".format(epoch+1,magnitude))
        print("getting initial fitnesses (This may take a while)",end="\r")
        prev_fitness = des_utils.get_fitness_set(kernels)[0]
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
                #print(best_kernel_text)
                #print(" "*cols,end="\r")
                print("   -> avg fitness:   {:.6f}".format(avg_fitness))
                print(" "*cols,end="\r")
                print("   -> best delta y:  {:.6f}".format(best_delta_y))
                print(" "*cols,end="\r")
                print("   -> worst delta y: {:.6f}".format(worst_delta_y))
                print(" "*cols,end="\r")
                print("   -> best delta x:  {:.6f}".format(best_delta_x))
                print(" "*cols,end="\r")
                print("   -> worst delta x: {:.6f}".format(worst_delta_x))
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
                    print("\033[F"*(15),end="")
                    #print(len(best_kernel_text),console_width,len(best_kernel_text)/(console_width-20))
                    #time.sleep(1)
                    #extra_rows = math.ceil(len(best_kernel_text)/(console_width)+1)
                    #print("\033[F"*extra_rows,end="")
            time_spend_reconstructing = 0
            time_spend_kerneling = 0

            
            children = des_utils.spawn_children_set(kernels,offspring_count,magnitude)

            #print(len(children)*len(children[0]),kernel_count)
            #print(children)
            eval_time = -time.perf_counter()
            #for child in children:
            #    child.insert(0,len(child))
            flattened_children = numpy.asarray(flatten_children(children,max_size),dtype=numpy.float32).flatten()
            fitness_results = cuda_tools.test_kernels(flattened_children,threads,256,thread_multiplier)
            eval_time += time.perf_counter()

            #print("eval done")
            child_fitness = deflatten_children(fitness_results,offspring_count,len(kernels))
            #print("sorting done")
            #time_spend_reconstructing = fitness_results[1]
            #time_spend_kerneling = fitness_results[2]
            #array_time = fitness_results[3]
            #worker_deploy_time = fitness_results[4]
            #worker_time = fitness_results[5]
            #fitness_time = fitness_results[6]
            #child_fitness = get_fitness_set(index,children,True)[0]

            selection_results = des_utils.select_fitess_member_set(kernels,prev_fitness,children,child_fitness)

            kernels = selection_results[0]
            prev_fitness = selection_results[1]

            max_fitness = selection_results[2]
            best_kernel = selection_results[3]
            avg_fitness =selection_results[4]

            end = time.perf_counter()
            gen_time = end-start

            kernel_deltas = des_utils.get_fitness(index,best_kernel,True)

            kernel_deltas[1].sort(key = abs)
            kernel_deltas[0].sort(key = abs)

            best_delta_x  = kernel_deltas[0][0]
            best_delta_y  = kernel_deltas[1][0]
            worst_delta_x = kernel_deltas[0][len(kernel_deltas)-1]
            worst_delta_y = kernel_deltas[1][len(kernel_deltas)-1]
            #if(generation>2):
            #    pool.close()
            #    exit()
        
        magnitude = magnitude/2

        best_kernels.append(best_kernel)
        best_fitnesses.append(max_fitness)
        average_fitness_preop.append(avg_fitness)

        sorted = des_utils.sort_kernels(kernels,prev_fitness)
        kernels = sorted[0]
        prev_fitness = sorted[1]
        median_fitness_preop.append(prev_fitness[math.floor(len(kernels)/2)])

        if operators!=None:
            if(len(operators) > epoch):
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

        sorted = des_utils.sort_kernels(kernels,prev_fitness)
        kernels = sorted[0]
        prev_fitness = sorted[1]
        median_fitness_postop.append(prev_fitness[math.floor(len(kernels)/2)])

        total_postop_fitness = 0
        for fitness in prev_fitness:
            total_postop_fitness+=fitness
        
        average_fitness_postop.append(total_postop_fitness/len(prev_fitness))

        print()

    print("evolution done.")
    #pool.close()

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
        log_file_name=log_file_name
        )
    
    kernel_file = open(kernel_file_name,"w")
    kernel_file.write(json.dumps(kernels,indent=4))
    kernel_file.close()

    return kernels
        
def continous_descent(stop_hour):
    i = 0
    now = datetime.datetime.now()
    kernels = []
    while(now.hour < stop_hour):
        i+=1
        print("The time is now {}:{}".format(now.hour,now.minute))
        print("ITERATION {}".format(i))
        print()
        now = datetime.datetime.now()
        kernels = descend("other/indexes/descent-data.json",
            max_reach=7,
            min_reach=4,
            generations=[100,100,100,100,100,100,100,100,100],
            epochs=9,
            offspring_count=4,
            initial_magnitude=1,
            sibling_multiplier=1,
            operators=[[prune,[0.07]],[prune,[0.07]],[prune,[0.07]],[prune,[0.07]],[prune,[0.07]],[prune,[0.07]],[prune,[0.07]]],
            initial_kernels = kernels,
            log_file_name="other/descent-logs/y-logs/descent-log-{}.log".format(i),
            kernel_file_name="other/descent-logs/y-logs/kernels-{}.log".format(i)
            )
        print()
        
    
    print("Time limit reached. Stopping Incarnations")
    print("The time is now {}:{}".format(now.hour,now.minute))



if __name__ == "__main__":
    print()
    #continous_descent(8)
    #exit()
    descend("other/indexes/descent-data.json",
            max_reach=6,
            min_reach=6,
            generations=[5,200,200,200,100,100,100,100],
            epochs=1,
            offspring_count=3,
            sibling_multiplier=1,
            operators=[[prune,[0.5]],[prune,[0.15]],[prune,[0.20]],[prune,[0.30]],[prune,[0.75]],[prune,[0.75]],None],
            initial_kernels = [],
            threads=2048
            )
    #data = (load_padded_data("other/indexes/descent-data.json",3))
    #angles = data["60"]["file-data"]['other/nav-logs/cm-60-woodpanel-maxbattery.json'][3]
    #left = data["60"]["data-sets"]['other/nav-logs/cm-60-woodpanel-maxbattery.json']["left"]
    #right = data["60"]["data-sets"]['other/nav-logs/cm-60-woodpanel-maxbattery.json']["right"]

    

    #print(nav.reconstruct(angles,left,right,False))

