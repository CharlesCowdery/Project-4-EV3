import math
import numpy as np
cimport numpy
import time
from libc.math cimport sin,cos,atan,sqrt



def reconstruct(l_rotations,r_rotations,int a_len):
    cdef double pi = math.pi
    cdef double distance_scalar = 29.240853516
    cdef double cross_section_length = 19.3

    deltas_l = []
    deltas_r = []

    cdef int k

    for k in range(1,a_len):
        deltas_l.append(l_rotations[k]-l_rotations[k-1])
        deltas_r.append(r_rotations[k]-r_rotations[k-1])
        
    
    cdef double x = 0
    cdef double y = 0
    cdef double angle = pi/2

    cdef double inside = 0
    cdef double outside = 0

    cdef double length = 0

    cdef double internal_radius = 0
    cdef double theta = 0
    cdef double mid_radius = 0
    cdef double raw_y = 0
    cdef double raw_x = 0
    cdef double raw_theta = 0

    cdef double l = 0
    cdef double r = 0

    cdef double delta = 0

    for i in range(a_len-1):
        l = deltas_l[i]
        r = deltas_r[i]
        inside = l*distance_scalar
        outside = r*distance_scalar

        raw_y = 0

        length = 0

        if(l != 0 and r != 0): 
            delta = outside-inside
            if(delta == 0):
                length = inside
            else:
                theta = delta/cross_section_length
                internal_radius = inside*cross_section_length/(delta)
                mid_radius = cross_section_length/2+internal_radius
                raw_x = cos(theta)*mid_radius-mid_radius
                raw_y = sin(theta)*mid_radius

                if(raw_x != 0):
                    #raw_y = sin(theta)*mid_radius   #might actually make it more accurate?
                    raw_theta = atan(raw_y/raw_x)
                else:
                    raw_theta = pi/2

                if raw_theta>0:
                    angle+=raw_theta-pi/2
                else:
                    angle+=pi/2+raw_theta

                length = sqrt(raw_x*raw_x + raw_y*raw_y)



        x+=length*cos(angle)
        y+=length*sin(angle)
    return (x,y)