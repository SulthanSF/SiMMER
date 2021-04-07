#!/usr/bin/env python

import rospy
import numpy
from scipy.optimize import minimize

class redundancy_resolver:

    def __init__(self, w, l_man, r_l0_mb_p0, m_t, r_G_tb, r_G_ee, x_i, bounds):
        self.w = w
        self.l_man = l_man
        self.r_l0_mb_p0 = r_l0_mb_p0
        self.m_t = m_t
        self.r_G_tb = r_G_tb
        self.r_G_ee = r_G_ee
        self.x_i = x_i
        self.bounds = bounds

    def objective(self, x):
        return (self.w[0]*(x[0]-self.x_i[0])**2+self.w[1]*(x[1]-self.x_i[1])**2+self.w[2]*(x[2]-self.x_i[2])**2+self.w[3]*(x[3]-self.x_i[3])**2+self.w[4]*(x[4]-self.x_i[4])**2)

    def ee_x_constraint(self, x):
        return x[3] + self.r_l0_mb_p0[0] + self.l_man[1]*numpy.sin(x[0])*numpy.sin(x[1]) + self.l_man[2]*numpy.sin(x[0])*numpy.sin(x[1]+x[2]) - self.r_G_ee[0]

    def ee_y_constraint(self, x):
        return x[4] + self.r_l0_mb_p0[1] - self.l_man[1]*numpy.sin(x[1])*numpy.cos(x[0]) - self.l_man[2]*numpy.cos(x[0])*numpy.sin(x[1]+x[2]) - self.r_G_ee[1]

    def ee_z_constraint(self, x):
        return self.r_G_tb[2] + self.r_l0_mb_p0[2] + self.l_man[0] + self.l_man[1]*numpy.cos(x[1]) + self.l_man[2]*numpy.cos(x[1]+x[2]) - self.r_G_ee[2]

    def ee_manipulability_constraint(self, x, m_t=0):
        if m_t is 0: m_t = self.m_t
        J = numpy.array([[numpy.cos(x[0])*(self.l_man[1]*numpy.sin(x[1])+self.l_man[2]*numpy.sin(x[1]+x[2])), numpy.sin(x[0])*(self.l_man[1]*numpy.cos(x[1])+self.l_man[2]*numpy.cos(x[1]+x[2])), self.l_man[2]*numpy.sin(x[0])*numpy.cos(x[1]+x[2])],
                         [numpy.sin(x[0])*(self.l_man[1]*numpy.sin(x[1])+self.l_man[2]*numpy.sin(x[1]+x[2])), -numpy.cos(x[0])*(self.l_man[1]*numpy.cos(x[1])+self.l_man[2]*numpy.cos(x[1]+x[2])), -self.l_man[2]*numpy.cos(x[0])*numpy.cos(x[1]+x[2])],
                         [0, -self.l_man[1]*numpy.sin(x[1])-self.l_man[2]*numpy.sin(x[1]+x[2]), -self.l_man[2]*numpy.sin(x[1]+x[2])]])
        manipulability = abs(numpy.linalg.det(J))/(((self.l_man[1]+self.l_man[2])/2)*self.l_man[1]*self.l_man[2])
        return manipulability-m_t

    def stroke_length_constraint_1(self, x):
        return numpy.hypot((x[3]-self.x_i[3]), (x[4]-self.x_i[4])) - 9*10**-5

    def stroke_length_constraint_2(self, x):
        return 1.25*10**-3 - numpy.hypot((x[3]-self.x_i[3]), (x[4]-self.x_i[4]))

    def resolve(self):
        x_0 = self.x_i
        constraint_1 = {'type': 'eq', 'fun': self.ee_x_constraint}
        constraint_2 = {'type': 'eq', 'fun': self.ee_y_constraint}
        constraint_3 = {'type': 'eq', 'fun': self.ee_z_constraint}
        constraint_4 = {'type': 'ineq', 'fun': self.ee_manipulability_constraint}
        constraint_5 = {'type': 'ineq', 'fun': self.stroke_length_constraint_1}
        constraint_6 = {'type': 'ineq', 'fun': self.stroke_length_constraint_2}
        cons = [constraint_1, constraint_2, constraint_3, constraint_4, constraint_5, constraint_6]
        solution = minimize(self.objective, x_0, method='SLSQP', bounds=self.bounds, constraints=cons)
        x = solution.x
        return x
