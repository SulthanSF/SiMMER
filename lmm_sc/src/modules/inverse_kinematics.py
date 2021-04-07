#!/usr/bin/env python

import rospy
import numpy

class ik_lmm_solver:

    def __init__(self, r_G_tb, eta_G_L0, r_G_ee, r_G_li_tip, r_l0_si_p0, li, di, r_l0_mb_p0, l_man, d_man, phi, phi_m, gamma_r, gamma_l):
        self.c1 = numpy.cos(eta_G_L0[0])
        self.s1 = numpy.sin(eta_G_L0[0])
        self.c2 = numpy.cos(eta_G_L0[1])
        self.s2 = numpy.sin(eta_G_L0[1])
        self.c3 = numpy.cos(eta_G_L0[2])
        self.s3 = numpy.sin(eta_G_L0[2])
        self.r_G_tb = r_G_tb
        self.r_G_ee = r_G_ee
        self.r_G_li_tip = r_G_li_tip
        self.r_l0_si_p0 = r_l0_si_p0
        self.li = li
        self.di_sum = sum(di)
        self.r_l0_mb_p0 = r_l0_mb_p0
        self.l_man = l_man
        self.d_man_sum = sum(d_man)
        self.phi = phi
        self.phi_m = phi_m
        self.gamma_r = gamma_r
        self.gamma_l = gamma_l
        self.A_G_L0 = numpy.eye(3, dtype=int)    # For current case when all eta_G_L0 = 0

    def solve_hexapod(self):
        leg_joint_angles = {}
        for i in range(6):
            leg = i+1
            r_G_pi3_si = -(self.r_G_tb+numpy.matmul(self.A_G_L0, self.r_l0_si_p0[leg])-self.r_G_li_tip[leg])
            ai = r_G_pi3_si[0]
            bi = r_G_pi3_si[1]
            ci = r_G_pi3_si[2]
            Ki1 = ai*self.c2*self.c3+bi*(self.c1*self.s3+self.s1*self.s2*self.c3)+ci*(self.s1*self.s3-self.c1*self.s2*self.c3)
            Ki2 = -ai*self.c2*self.s3+bi*(self.c1*self.c3-self.s1*self.s2*self.s3)+ci*(self.s1*self.c3+self.c1*self.s2*self.s3)
            Ki3 = ai*self.s2-bi*self.s1*self.c2+ci*self.c1*self.c2
            Ki4 = numpy.sqrt(Ki1**2+Ki2**2-self.di_sum**2)
            Ki5 = ((Ki3-self.li[0]*numpy.sin(self.phi))**2+(Ki4-self.li[0]*numpy.cos(self.phi))**2-self.li[1]**2-self.li[2]**2)/(2*self.li[1]*self.li[2])
            if leg in [1, 3, 5]:
                leg_joint_angles['joint_' + str(leg) + '_1'] = self.gamma_l-(2*numpy.arctan((Ki1+Ki4)/(self.di_sum+Ki2)))
                leg_joint_angles['joint_' + str(leg) + '_2'] = -(self.phi-2*numpy.arctan(((Ki3-self.li[0]*numpy.sin(self.phi))+numpy.sqrt((Ki3-self.li[0]*numpy.sin(self.phi))**2+(Ki4-self.li[0]*numpy.cos(self.phi))**2-(self.li[1]+self.li[2]*Ki5)**2))/(self.li[1]+self.li[2]*Ki5+Ki4-self.li[0]*numpy.cos(self.phi))))
                leg_joint_angles['joint_' + str(leg) + '_3'] = -2*numpy.arctan(numpy.sqrt((1-Ki5)/(1+Ki5)))
            elif leg in [2, 4, 6]:
                leg_joint_angles['joint_' + str(leg) + '_1'] = self.gamma_r-(2*numpy.arctan((Ki1-Ki4)/(self.di_sum+Ki2)))
                leg_joint_angles['joint_' + str(leg) + '_2'] = (self.phi-2*numpy.arctan(((Ki3-self.li[0]*numpy.sin(self.phi))+numpy.sqrt((Ki3-self.li[0]*numpy.sin(self.phi))**2+(Ki4-self.li[0]*numpy.cos(self.phi))**2-(self.li[1]+self.li[2]*Ki5)**2))/(self.li[1]+self.li[2]*Ki5+Ki4-self.li[0]*numpy.cos(self.phi))))
                leg_joint_angles['joint_' + str(leg) + '_3'] = +2*numpy.arctan(numpy.sqrt((1-Ki5)/(1+Ki5)))
        return leg_joint_angles

    def solve_manipulator(self):
        man_joint_angles = {}
        r_G_ee_mb = -(self.r_G_tb+numpy.matmul(self.A_G_L0, self.r_l0_mb_p0)-self.r_G_ee)
        a = r_G_ee_mb[0]
        b = r_G_ee_mb[1]
        c = r_G_ee_mb[2]
        K1 = a*self.c2*self.c3+b*(self.c1*self.s3+self.s1*self.s2*self.c3)+c*(self.s1*self.s3-self.c1*self.s2*self.c3)
        K2 = -a*self.c2*self.s3+b*(self.c1*self.c3-self.s1*self.s2*self.s3)+c*(self.s1*self.c3+self.c1*self.s2*self.s3)
        K3 = a*self.s2-b*self.s1*self.c2+c*self.c1*self.c2
        K4 = numpy.sqrt(K1**2+K2**2-self.d_man_sum**2)
        K5 = ((K3-self.l_man[0]*numpy.sin(self.phi_m))**2+(K4-self.l_man[0]*numpy.cos(self.phi_m))**2-self.l_man[1]**2-self.l_man[2]**2)/(2*self.l_man[1]*self.l_man[2])
        man_joint_angles['joint_m_1'] = -(2*numpy.arctan((-K2+K4)/(self.d_man_sum+K1)))
        man_joint_angles['joint_m_2'] = -(self.phi_m-2*numpy.arctan(((K3-self.l_man[0]*numpy.sin(self.phi_m))+numpy.sqrt((K3-self.l_man[0]*numpy.sin(self.phi_m))**2+(K4-self.l_man[0]*numpy.cos(self.phi_m))**2-(self.l_man[1]+self.l_man[2]*K5)**2))/(self.l_man[1]+self.l_man[2]*K5+K4-self.l_man[0]*numpy.cos(self.phi_m))))
        man_joint_angles['joint_m_3'] = -2*numpy.arctan(numpy.sqrt((1-K5)/(1+K5)))
        return man_joint_angles
        
    def solve_lmm(self):
        leg_joint_angles = self.solve_hexapod()
        man_joint_angles = self.solve_manipulator()
        lmm_joint_angles = leg_joint_angles
        lmm_joint_angles.update(man_joint_angles)
        return lmm_joint_angles
