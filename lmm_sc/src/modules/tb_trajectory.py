# TB trajectory

import numpy

def tb_trajectory(theta_c, vel_tb_max, tb_pos_i, tb_time, T_in):
    D_x_G_p0_oi = 0*numpy.cos(theta_c)
    D_x_G_p0_of = vel_tb_max*numpy.cos(theta_c)
    D_y_G_p0_oi = 0*numpy.sin(theta_c)
    D_y_G_p0_of = vel_tb_max*numpy.sin(theta_c)
    D_z_G_p0_oi = 0
    D_z_G_p0_of = 0

    rdot_G_p0_o_i = numpy.array([D_x_G_p0_oi, D_y_G_p0_oi, D_z_G_p0_oi])
    rdot_G_p0_o_f = numpy.array([D_x_G_p0_of, D_y_G_p0_of, D_z_G_p0_of])

    a_vel = rdot_G_p0_o_f-rdot_G_p0_o_i

    N1 = int(numpy.ceil(tb_time[3]/T_in))
    D_del_a = 1/(tb_time[1]-tb_time[0])
    D_del_d = 1/(tb_time[3]-tb_time[2])

    r_G_p0_ot = []

    del_a = (tb_time[1]-tb_time[0])/(tb_time[1]-tb_time[0])
    r_G_p0_ot1 = (tb_pos_i+((rdot_G_p0_o_i*del_a)+(a_vel)*((del_a)**3*(1-del_a/2)))*(1/(D_del_a)))    # tb pos at t1
    r_G_p0_ot2 = (r_G_p0_ot1+(rdot_G_p0_o_f)*(tb_time[2]-tb_time[1]))                                 # tb pos at t2

    for N in range(N1):
        t = (N+1)*T_in
        if (t>=tb_time[0] and t<=tb_time[1]):                   # From t0 to t1
            del_a = (t-tb_time[0])/(tb_time[1]-tb_time[0])
            r_G_p0_ot11 = (tb_pos_i+((rdot_G_p0_o_i*del_a)+(a_vel)*((del_a)**3*(1-del_a/2)))*(1/(D_del_a)))
            r_G_p0_ot.append(r_G_p0_ot11)
        elif (t>tb_time[1] and t<=tb_time[2]):                  # From t1 to t2
            r_G_p0_ot11 = (r_G_p0_ot1+(rdot_G_p0_o_f)*(t-tb_time[1]))
            r_G_p0_ot.append(r_G_p0_ot11)
        elif (t>tb_time[2] and t<=tb_time[3]):                  # From t2 to t3
            del_d = (t-tb_time[2])/(tb_time[3]-tb_time[2])
            r_G_p0_ot11 = (r_G_p0_ot2+((rdot_G_p0_o_f*del_d)-(a_vel*(del_d**3*(1-del_d/2))))*(1/(D_del_d)))
            r_G_p0_ot.append(r_G_p0_ot11)
        else:                                                   # Ending at t3
            del_d = (tb_time[3]-tb_time[2])/(tb_time[3]-tb_time[2])
            r_G_p0_ot11 = (r_G_p0_ot2+((rdot_G_p0_o_f*del_d)-(a_vel*(del_d**3*(1-del_d/2))))*(1/(D_del_d)))
            r_G_p0_ot.append(r_G_p0_ot11)

    return r_G_p0_ot
