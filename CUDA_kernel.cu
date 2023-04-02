
#define FIELD_COUNT 1//%field_count%
#define SUB_FIELD_SIZE 1//%field_size%
#define FLAT_INDEX_SIZE 1//%index_size%
#define KERNEL_FRAME_SIZE 1//%kernel_size%
#define DISTANCE_SCALAR 1//%distance_scalar%
#define CROSS_SECTION 1//%cross_section_length%
#define KERNEL_COUNT 1//%kernel_load%

#define KERNEL_ARRAY_SIZE KERNEL_COUNT*KERNEL_FRAME_SIZE+KERNEL_COUNT
#define PI 3.141592

float index[FIELD_COUNT][2][SUB_FIELD_SIZE];
float final_positions[FIELD_COUNT][2];
float kernel_data[KERNEL_COUNT][KERNEL_FRAME_SIZE+1];

extern "C"{


__global__ void initialize_index(float index_data[FLAT_INDEX_SIZE],float flat_results[FIELD_COUNT*2]){
    int position = 0;
    for(int field_index = 0; field_index < FIELD_COUNT; field_index++){

        int field_length = index_data[position];
        int starting_position = position;

        for(int sub_field_index = 0; sub_field_index<2;sub_field_index++){
            index[field_index][sub_field_index][0] = field_length;
            position++;
            for(int i = 1; i < field_length+1; i++){
                index[field_index][sub_field_index][i] = index_data[position];
                position++;
            }
            position = starting_position+SUB_FIELD_SIZE;
            final_positions[field_index][sub_field_index] = flat_results[field_index*2+sub_field_index];
        }
        position = starting_position+SUB_FIELD_SIZE*2;
    }
}

__global__ void apply_kernel(int field_index, float kernel[KERNEL_FRAME_SIZE],int kernel_size,
                             float output_left[SUB_FIELD_SIZE], float output_right[SUB_FIELD_SIZE]){
    //float field[2][SUB_FIELD_SIZE];
    //*field[0] = *index[field_index][0];
    //*field[1] = *index[field_index][1];
    int data_length = index[field_index][0][0];

    int kernel_start_position = (KERNEL_FRAME_SIZE-kernel_size)/2;
    int kernel_end_position   =  KERNEL_FRAME_SIZE-(KERNEL_FRAME_SIZE-kernel_size)/2;
    int half_kernel_size      =  floor((double) KERNEL_FRAME_SIZE/2);
    int data_end_index        =  data_length-half_kernel_size;

    output_left[0] = data_length;
    output_right[0] = data_length;

    for(int center_index = 1+half_kernel_size; center_index<data_end_index; center_index++){
        float accumulator_1 = 0;
        float accumulator_2 = 0;

        for(int kernel_index = kernel_start_position;
            kernel_index<kernel_end_position;
            kernel_index++){

            int data_index = center_index+kernel_index-half_kernel_size;
            if(data_index>0 && data_index<=data_length){
                float kernel_value = kernel[kernel_index];
                accumulator_1+=index[field_index][0][data_index]*kernel_value;
                accumulator_2+=index[field_index][1][data_index]*kernel_value;
            } 
        }
        //output_left[center_index] = index[field_index][0][center_index];
        output_left[center_index] = accumulator_1;
        //output[center_index+data_length] = index[field_index][0][center_index];
        output_right[center_index] = accumulator_2;
    }   
}

__global__ void model_turn_circle(float l,float r,float output[2]){
    float inside = l*DISTANCE_SCALAR;
    float outside = r*DISTANCE_SCALAR;
    float length = inside;
    float raw_theta = 0;
    if(inside == 0){
        inside  = 0.0000001;
    }
    if(outside == 0){
        outside = 0.0000001;
    }

    float delta = outside-inside;
    if(l!=0 && r != 0){
    if(delta!=0){
        float internal_radius = inside*CROSS_SECTION/delta;
        float theta = delta/CROSS_SECTION;
        float mid_radius = CROSS_SECTION/2.0+internal_radius;
        float x = cosf(theta)*mid_radius-mid_radius;
        float y = sinf(theta)*mid_radius;

        raw_theta = atan2f(y,x)-PI/2.0;

        length = sqrtf(x*x+y*y);
        
    }
    }else{
        length = 0;
    }
    output[0] = raw_theta;
    output[1] = length;
    
}

void reconstruct(float left[SUB_FIELD_SIZE], float right[SUB_FIELD_SIZE], float output[2]){
    float delta_left[SUB_FIELD_SIZE];
    float delta_right[SUB_FIELD_SIZE];
    float padding_size = floorf(KERNEL_FRAME_SIZE/2);
    int data_length = left[0];
    int starting_index = padding_size+1;
    int end_index = data_length-padding_size;

    for(int i = starting_index+1; i < end_index; i++){
        delta_left[i] = left[i]-left[i-1];
        delta_right[i] = right[i]-right[i-1];
    }

    float x = 0;
    float y = 0;
    float angle = PI/2;

    float prediction_output[2];
    for(int i = starting_index; i < end_index; i++){
        model_turn_circle(delta_left[i],delta_right[i],prediction_output);
        angle+=prediction_output[0];

        x+=prediction_output[1]*cosf(angle);
        y+=prediction_output[1]*sinf(angle);
    }

    output[0]=x;
    output[1]=y;
}


float get_kernel_fitness(int kernel_index){
    float kernel[KERNEL_FRAME_SIZE];
    int kernel_size = kernel_data[kernel_index][0];
    for(int i = 0; i < KERNEL_FRAME_SIZE;i++){
        kernel[i] = kernel_data[kernel_index][i+1];
    }
    int stopping_index = 1;//FIELD_COUNT-10;
    float delta_x = 0;
    float delta_y = 0;
    float left_array[SUB_FIELD_SIZE];
    float right_array[SUB_FIELD_SIZE];
    float reconstruction_output[2] = {0,0};
    float total_deltas[2] = {0,0};
    for(int field_index = 0; field_index<stopping_index;field_index++){
        apply_kernel(field_index,kernel,kernel_size,left_array,right_array);
        reconstruct(left_array,right_array,reconstruction_output);
        delta_x = reconstruction_output[0]-final_positions[field_index][0];
        delta_y = reconstruction_output[1]-final_positions[field_index][1];
        total_deltas[0] += fabsf(delta_x);//
        total_deltas[1] += fabsf(delta_y);
    }  
    return 1/(powf(total_deltas[1],3)+powf(total_deltas[1],2)); 
}

__global__ void evaluate_kernels(float kernels[KERNEL_COUNT*KERNEL_FRAME_SIZE+KERNEL_COUNT],float output[KERNEL_COUNT],int thread_multiplier){
    unsigned int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if(threadIdx.x==0){
        for(int k = 0; k < KERNEL_COUNT; k++){
            for(int i = 0; i < KERNEL_FRAME_SIZE+1; i++){
                kernel_data[k][i] = kernels[k*(KERNEL_FRAME_SIZE+1)+i];
            }
        }
    }

    __syncthreads();

    int end_index = (tid+1)*thread_multiplier;
    int start_index = tid*thread_multiplier;
    for(int kernel_index = start_index; kernel_index < end_index; kernel_index++){
        output[kernel_index]=get_kernel_fitness(kernel_index);
    }
}

__global__ void test_multiply(const float* x1, const float* x2, float* y, \
                              unsigned int N)
{
    unsigned int tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid < N)
    {
        y[tid] = x1[tid] * x2[tid];
    }
}

}