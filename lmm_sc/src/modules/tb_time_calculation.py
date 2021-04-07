# TB time calculation

import numpy
from scipy.optimize import fsolve

def tb_time_calculation(r_G_tb_i, r_G_tb_f, theta_c, vel_tb_max, T_stroke, T_in):
    t_0 = 0
    t_3 = t_0 + T_stroke
    del_G_tb = r_G_tb_f-r_G_tb_i
    D_x_G_p0_oi = 0*numpy.cos(theta_c)
    D_x_G_p0_of = vel_tb_max*numpy.cos(theta_c)
    D_y_G_p0_oi = 0*numpy.sin(theta_c)
    D_y_G_p0_of = vel_tb_max*numpy.sin(theta_c)
    D_z_G_p0_oi = 0
    D_z_G_p0_of = 0
    rdot_G_p0_o_i = numpy.array([D_x_G_p0_oi, D_y_G_p0_oi, D_z_G_p0_oi])
    rdot_G_p0_o_f = numpy.array([D_x_G_p0_of, D_y_G_p0_of, D_z_G_p0_of])
    a_vel = rdot_G_p0_o_f-rdot_G_p0_o_i

    Eq = lambda t1 : (r_G_tb_f[0]-(r_G_tb_i[0]+((rdot_G_p0_o_i*((T_in-t_0)/(t1-t_0)))+(a_vel)*(((T_in-t_0)/(t1-t_0))**3*(1-((T_in-t_0)/(t1-t_0))/2)))[0]*(1/(1/(t1-t_0)))))

    t1_guess = 0.1
    sol = fsolve(Eq, t1_guess)

    t_1 = t_0 + sol
    t_2 = t_3 - sol
    tb_time = [t_0, t_1, t_2, t_3]

    return tb_time
