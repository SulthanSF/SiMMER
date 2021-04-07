#!/usr/bin/env python

# Import required libraries
import rospy
import numpy
import inverse_kinematics

def lmm_joint_angles_init(r_G_tb_init, r_G_ee_init, x_si_li_init, eta_G_L0, r_l0_si_p0, li, di, r_l0_mb_p0, l_man, d_man, phi, phi_m, gamma_r, gamma_l):
    r_G_li_tip_init = {}
    for i in range(6):
        leg = i+1
        r_G_li_tip_init[leg] = r_G_tb_init + r_l0_si_p0[leg] + numpy.array([((-1)**leg)*x_si_li_init, 0, (-r_G_tb_init[2]-r_l0_si_p0[leg][2])])
    ik_lmm = inverse_kinematics.ik_lmm_solver(r_G_tb_init, eta_G_L0, r_G_ee_init, r_G_li_tip_init, r_l0_si_p0, li, di, r_l0_mb_p0, l_man, d_man, phi, phi_m, gamma_r, gamma_l)
    lmm_joint_angles_init = ik_lmm.solve_lmm()
    return lmm_joint_angles_init, r_G_li_tip_init
