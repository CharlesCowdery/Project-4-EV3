import cupy as cp  # Subject to change before release
import numpy as np
import copy
import math
import descent_utils as des_utils
import constants_r as con
import navigation_r as nav
import navperf

kernel_code = ""
index = None
field_reduced_index = []
flattened_index = []
largest_field = 0
padded_field_size = 0
maximum_kernel_size = 0
GPU_module = None
kernel_count = 2
flat_results = []

kernel_evaluation_function = None

def exitor():
    print("exiting...")
    exit()

def reduce_index(index):
    global field_reduced_index, largest_field, flat_arguments
    field_reduced_index = []
    for catagory_name in index:
        catagory = index[catagory_name]
        for file_name in catagory["data-sets"]:
            data = catagory["data-sets"][file_name]
            entry = []
            entry.append(data["left"])
            entry.append(data["right"])
            field_reduced_index.append(copy.deepcopy(entry))
            expected_x = catagory["file-data"][file_name][4]
            expected_y = catagory["file-data"][file_name][5]
            flat_results.append(expected_x)
            flat_results.append(expected_y)
            if len(data["left"])>largest_field:
                largest_field=len(data["right"])

def flatten_index(maximum_kernel_size):
    global flattened_index, padded_field_size
    final = []
    half_kernel_size = math.floor(maximum_kernel_size/2)
    padded_field_size = largest_field+half_kernel_size*2+1#adding padding plus size data
    total_field_size = padded_field_size*2-1#two fields, but they share a size data
    for i in range(len(field_reduced_index)):

        temp = []

        field = field_reduced_index[i] #get field

        field_lasts = [field[0][len(field[0])-1],field[1][len(field[1])-1]]

        field_length = len(field[0].tolist())+half_kernel_size*2+1   #get length + padding + length indicator
        
        temp+=[field_length]           #add length data
        temp+=[field[0][0]]*half_kernel_size     #add padding
        temp+=field[0].tolist()        #add first field data
        temp+=[field_lasts[0]]*half_kernel_size     #add padding for both
        temp+=[0]*(padded_field_size-field_length)#compesate for non data

        temp+=[field_length]           #add length data
        temp+=[field[0][0]]*half_kernel_size     #add padding
        temp+=field[1].tolist()        #add second field data
        temp+=[field_lasts[0]]*half_kernel_size     #add padding
        temp+=[0]*(padded_field_size-field_length)#compesate for non data

        temp+=[0]*(total_field_size-len(temp)) #fill out the rest of the data space

        final+=temp
    
    flattened_index = final

    


def prep_code(code):
    code = code.replace("1//%field_count%",str(len(field_reduced_index)))
    code = code.replace("1//%field_size%",str(padded_field_size))
    code = code.replace("1//%index_size%",str(len(flattened_index)))
    code = code.replace("1//%kernel_size%",str(maximum_kernel_size))
    code = code.replace("1//%distance_scalar%",str(con.distance_scalar))
    code = code.replace("1//%cross_section_length%",str(con.cross_section_length))
    code = code.replace("1//%kernel_load%",str(kernel_count))
    return code

def initialize(index,_maximum_kernel_size,_kernel_count):
    global maximum_kernel_size, GPU_module,kernel_evaluation_function
    global kernel_code,kernel_count
    maximum_kernel_size = _maximum_kernel_size
    kernel_count = _kernel_count

    print("Initializing CUDA workflow")

    print(" -reducing index...     ",end="")
    reduce_index(index)
    print("Done!")

    print(" -flattening index...   ",end="")
    flatten_index(maximum_kernel_size)
    print("Done!")
    #print(" -Flat index length: {}".format(len(flattened_index)))

    print(" -preping kernel...     ",end="")
    code = open("CUDA_kernel.cu","r").read()
    code = prep_code(code)
    kernel_code = code
    print("Done!")
    
    #print(kernel_code)


    print(" -creating module...    ",end="")
    GPU_module = cp.RawModule(code=kernel_code)
    print("Done!")


    print(" -deploying index...    ",end="")
    flattened_index_cp = cp.asarray(flattened_index,dtype=cp.float32)
    flat_results_cp = cp.asarray(flat_results,dtype=cp.float32)
    index_initializer = GPU_module.get_function('initialize_index')
    index_initializer((1,),(1,),(flattened_index_cp,flat_results_cp,))
    print("Done!")

    kernel_evaluation_function = GPU_module.get_function("evaluate_kernels")

def test_kernels(kernels,thread_count,block_size,multiplier):
    block_count = math.floor(thread_count/block_size)
    fitness_out = cp.zeros((kernel_count,),dtype=cp.float32)
    cp_kernels = cp.asarray(kernels,dtype=cp.float32)
    kernel_evaluation_function((block_count,),(block_size,),(cp_kernels,fitness_out,cp.int32(multiplier),))
    return fitness_out.tolist()

def main():
    
    index_file_name = "other/indexes/descent-data.json"
    index = des_utils.load_padded_data(index_file_name,0)
    initialize(index,5)
    print("init done")
    kerneler = GPU_module.get_function("apply_kernel")
    modeler = GPU_module.get_function("model_turn_circle")

    kernel_array = [3,0,0,1,0,0,5,0,0,1,0,0]
    kernel_array_cp = cp.asarray(kernel_array,dtype=cp.float32)

    output_array = cp.zeros((2,),dtype=cp.float32)

    print("testing evaluator")
    #kernel_evaluator((1,),(1,),(kernel_array_cp,output_array))
    print("eval done")
    print(kernel_array_cp)
    print(output_array)

    out = cp.zeros((2,),dtype=cp.float32)
    modeler((1,),(1,),(cp.float32(0.5),cp.float32(0.25),out,))
    print("modeler results: "+str(out.tolist()))

    print("recon results:"+str(navperf.reconstruct(field_reduced_index[0][0],field_reduced_index[0][1],len(field_reduced_index[0][0]))))



    """
    out_left = cp.zeros((padded_field_size,),dtype=cp.float32)
    out_right = cp.zeros((padded_field_size,),dtype=cp.float32)

    kernel = [-1,2,-1]

    field_index=0
    print("testing kerneler")
    kerneler((1,),(1,),(field_index,cp.asarray(kernel,dtype=cp.float32),3,out_left,out_right))

    verify = np.convolve(field_reduced_index[0][0],kernel).tolist()

    out = out_left.tolist()

    chosen_field = verify
    offset_index = 0#padded_field_size*5
    """
    print("test results:")
    #for i in range(flattened_index[0+offset_index]-1):
    #    print("{:10.7f}".format(out[i]),"{:10.7f}".format(chosen_field[i+offset_index]))


    print(len(field_reduced_index))
    exitor()
    N = 10
    x1 = cp.arange(N**2, dtype=cp.float32).reshape(N, N)
    x2 = cp.ones((N, N), dtype=cp.float32)
    y = cp.zeros((N, N), dtype=cp.float32)
    print(y)
    ker_times((N,), (N,), (x1, x2, y, N**2)) # y = x1 * x2
    assert cp.allclose(y, x1 * x2)
    print(y)
    print(x1)
    print(x2)



if __name__ == "__main__":
    main()
