import math
circumference_scalar = 0.64*1.66666666
distance_scalar = 29.240853516

cross_section_length = 19.3 #cm

wheel_diameter = 3.3 #cm
wheel_circumference = 2*math.pi*wheel_diameter*circumference_scalar

kernel_x = [
            -0.562695605407139,
            -0.5962977621462894,
            1.1121001111391162,
            0.13306742061725702,
            -1.1808440591592888,
            4.363727089044287,
            1.3687705574915576,
            1.330276268222195,
            -3.8201362564941888
        ]
kernel_y = [
            -0.030550588530728187,
            -0.29899923732899797,
            -0.1567837915659996,
            -0.10169194744346285,
            -0.08029518362853227,
            0.11450057032458848,
            0.4249561597397771
        ]