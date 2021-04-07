#!/usr/bin/env python

# Import required libraries
import numpy

# Body Dimensions
di = [10**-10, 0, 0]
d_man = [10**-10, 0, 0]
li = [0.095, 0.105, 0.100]
l_man = [0.0395, 0.250, 0.290]
phi = 0
phi_m = numpy.pi/2

# Radius vectors from TB to Hip Joints
r_l0_si_p0 = {1:numpy.array([-0.08, 0.205, 0.0155]),
              2:numpy.array([0.08, 0.205, 0.0155]),
              3:numpy.array([-0.08, 0, .0155]),
              4:numpy.array([0.08, 0, 0.0155]),
              5:numpy.array([-0.08, -0.205, 0.0155]),
              6:numpy.array([0.08, -0.205, 0.0155])}
r_l0_mb_p0 = numpy.array([0, 0.1025, 0.0425])

# Maximum velocity condition of TB
vel_tb_max = 0.04              #Can be varied for improvement

# Formulated dimensions
gamma_r = 0
gamma_l = 0
eta_G_L0 = [0, 0, 0]

# Inputs for time function
f_in = 10
T_in = 1.0/f_in
T_stroke = 1

# Inputs for leg tip trajectory local
Hmi1 = 0.015
del_h = 0.002
hi3_dash = 0
lt_time_ratio = 0.2

# Manipulability threshold to decide whether to move TB
m_t_man = 0.25

# Inputs for redundancy resolution
w = [1,1,1,20,20]
m_t = 0.25
bounds = ((-1.5708,1.5708),(-1.5708,1.5708),(-1.5708,1.5708),(-numpy.inf,numpy.inf),(-numpy.inf,numpy.inf))

# Initialize the lmm
r_G_tb_init = numpy.array([0, 0, 0.125])
r_G_ee_init = numpy.array([0, 0.45, 0.55])
x_si_li_init = 0.175
