#!/usr/bin/env python

# Import required libraries
import rospy
import numpy

class leg_tip_traj_local:

    def __init__(self, support_legs, swing_length, theta_c, T_stroke, T_in, Hmi1, del_h, hi3_dash, lt_time_ratio):
        self.support_legs = support_legs
        self.swing_length = swing_length
        self.theta_c = theta_c
        self.T_in = T_in
        self.Hmi1 = Hmi1
        self.del_h = del_h
        self.hi3_dash = hi3_dash
        self.lt_time = [0, T_stroke*lt_time_ratio, T_stroke*(1-lt_time_ratio), T_stroke]
        self.rot_crab = numpy.array([[numpy.cos(theta_c), -numpy.sin(theta_c), 0],
                                     [numpy.sin(theta_c), numpy.cos(theta_c), 0],
                                     [0, 0, 1]])

    def full_support_leg(self):
        ts_start = self.lt_time[0]
        ts_end = self.lt_time[3]
        n = int(numpy.ceil((ts_end-ts_start)/self.T_in))
        r_ei11 = numpy.zeros((n, 3))
        r_ei11 = r_ei11.tolist()
        return r_ei11

    def full_swing_leg(self):
        ts_start = self.lt_time[0]
        ts_end = self.lt_time[3]
        pi3 = numpy.array([0, 0, 0])
        del_pi3 = numpy.array([self.swing_length, 0, self.hi3_dash])
        pi3_dash = pi3 + del_pi3
        N1 = int(numpy.ceil((ts_end-ts_start)/self.T_in))

        # Trajectory along X-axis
        x_ei_t0s = pi3[0]
        x_ei_t3s = pi3_dash[0]
        a_xi_pp_dash = x_ei_t3s - x_ei_t0s
        x_ei11 = []
        for N in range(N1):
            t = (N+1)*self.T_in
            if (t<=self.lt_time[3]):
                del_pp_dash = (t-self.lt_time[0])/(self.lt_time[3]-self.lt_time[0])
                D_del_pp_dash = 1/(self.lt_time[3]-self.lt_time[0])
                x_ei1 = x_ei_t0s+a_xi_pp_dash*(del_pp_dash)**2*(3-2*del_pp_dash)
                x_ei11.append(x_ei1)
            else:
                x_ei2 = x_ei_t3s
                x_ei11.append(x_ei2)

        # Trajectory for Y-axis
        y_ei11 = [0] * (N1)

        # Trajectory for Z-axis
        z_L_Qi3 = self.Hmi1 + self.del_h
        z_L_Ti3 = z_L_Qi3
        z_ei_t0s = pi3[2]
        z_ei_t1s = z_L_Qi3
        z_ei_t2s = z_L_Ti3
        z_ei_t3s = pi3_dash[2]
        a_zi_pQ = z_ei_t1s - z_ei_t0s
        a_zi_Tp_dash = z_ei_t3s - z_ei_t2s
        z_ei11 = []
        for N in range(N1):
            t = (N+1)*self.T_in
            if (t>=self.lt_time[0] and t<self.lt_time[1]):
                del_pQ = (t-self.lt_time[0])/(self.lt_time[1]-self.lt_time[0])
                z_e1 = z_ei_t0s+a_zi_pQ*(del_pQ)**2*(3-2*del_pQ)
                z_ei11.append(z_e1)
            elif (t>=self.lt_time[1] and t<=self.lt_time[2]):
                z_e2 = z_ei_t1s
                z_ei11.append(z_e2)
            elif (t>self.lt_time[2] and t<=self.lt_time[3]):
                del_Tp_dash = (self.lt_time[3]-t)/(self.lt_time[3]-self.lt_time[2])
                z_e3 = z_ei_t3s-a_zi_Tp_dash*del_Tp_dash**2*(3-2*del_Tp_dash)
                z_ei11.append(z_e3)
            else:
                z_e4 = z_ei_t3s
                z_ei11.append(z_e4)
        r_ei11_T = numpy.array([x_ei11, y_ei11, z_ei11])
        r_ei11 = r_ei11_T.T
        r_ei11 = r_ei11.tolist()
        return r_ei11

    def calculate_trajectry(self):
        leg_tip_trajectory_local = {}
        for i in range(6):
            leg = i+1
            if leg in self.support_legs:
                traj = numpy.array(self.full_support_leg(), dtype=float)
                rot_traj = numpy.matmul(self.rot_crab, traj.T).T
                leg_tip_trajectory_local[leg] = rot_traj.tolist()
            else:
                traj = numpy.array(self.full_swing_leg(), dtype=float)
                rot_traj = numpy.matmul(self.rot_crab, traj.T).T
                leg_tip_trajectory_local[leg] = rot_traj.tolist()
        return leg_tip_trajectory_local
