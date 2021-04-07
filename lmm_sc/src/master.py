#!/usr/bin/env python

# ROS node for Real-Time motion planning

# Import required libraries
import rospy
import rospkg
import tf2_ros
import numpy
import time
import csv
import os
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

# Add modules path to sys
import sys
rospack = rospkg.RosPack()
package_path = rospack.get_path('lmm_sc')
sys.path.insert(0, package_path + '/src/modules')

# Import modules
import inputs                                   # Inputs
import tb_time_calculation                      # TB time calculation
import tb_trajectory                            # TB trajectory
import leg_tip_traj_local                       # Leg Tip Trajectory Local
import leg_tip_traj_local_to_global             # Leg Tip Trajectory Global
import redundancy_resolution                    # Redundancy Resolution
import inverse_kinematics                       # Inverse Kinematics
import initialize_lmm                           # Initialize LMM

# Initialize master node
rospy.init_node('lmm_sc_master')

# Initialize TF
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rospy.sleep(1)

# Global variables
is_tb_in_motion = False
traj_r_G_tb = []
traj_r_G_leg_tip = {1:[], 2:[], 3:[], 4:[], 5:[], 6:[]}
r_G_tb = inputs.r_G_tb_init
r_G_ee = inputs.r_G_ee_init
r_G_leg_tip = {}
lmm_joint_angles = {}
stroke_length = 0
time_instance = 0

# For Recording Data to CSV
data_folder = package_path + '/data/'
if not os.path.exists(data_folder):
    os.makedirs(data_folder)
data_file_name = time.strftime('%Y_%m_%d__%H_%M_%S') + '.csv'
data_file = data_folder + data_file_name

# Define the publisher
pub = rospy.Publisher('lmm_joint_states', JointState, queue_size = 10)

# Initialize LMM
rospy.sleep(1)
lmm_joint_angles, r_G_leg_tip = initialize_lmm.lmm_joint_angles_init(inputs.r_G_tb_init, inputs.r_G_ee_init, inputs.x_si_li_init, inputs.eta_G_L0, inputs.r_l0_si_p0, inputs.li, inputs.di, inputs.r_l0_mb_p0, inputs.l_man, inputs.d_man, inputs.phi, inputs.phi_m, inputs.gamma_r, inputs.gamma_l)
lmm_joint_states = JointState()
lmm_joint_states.header.stamp = rospy.Time.now()
for joint, angle in lmm_joint_angles.items():
    lmm_joint_states.name.append(joint)
    lmm_joint_states.position.append(angle)
pub.publish(lmm_joint_states)

# Callback for Subscriber
def callback(lmm_inputs):
    start_time = time.time()
    global is_tb_in_motion
    global traj_r_G_tb
    global traj_r_G_leg_tip
    global r_G_tb
    global r_G_ee
    global r_G_leg_tip
    global inputs
    global pub
    global lmm_joint_angles
    global stroke_length
    global time_instance
    global data_file
    ee_increments = numpy.array(lmm_inputs.data)
    time_instance += 1

    # Check TB motion
    if not traj_r_G_tb:
        is_tb_in_motion = False
    else:
        is_tb_in_motion = True

    # Check EE inputs
    if all(ee_increment == 0 for ee_increment in ee_increments):
        is_inputs_zero = True
    else:
        is_inputs_zero = False

    if is_tb_in_motion:
        r_G_tb = traj_r_G_tb.pop(0)
        r_G_ee = r_G_ee + ee_increments
        r_G_leg_tip = {}
        for i in range(6):
            leg = i+1
            r_G_leg_tip[leg] = traj_r_G_leg_tip[leg].pop(0)
        ik_lmm = inverse_kinematics.ik_lmm_solver(r_G_tb, inputs.eta_G_L0, r_G_ee, r_G_leg_tip, inputs.r_l0_si_p0, inputs.li, inputs.di, inputs.r_l0_mb_p0, inputs.l_man, inputs.d_man, inputs.phi, inputs.phi_m, inputs.gamma_r, inputs.gamma_l)
        lmm_joint_angles = ik_lmm.solve_lmm()
        lmm_joint_states = JointState()
        lmm_joint_states.header.stamp = rospy.Time.now()
        for joint, angle in lmm_joint_angles.items():
            lmm_joint_states.name.append(joint)
            lmm_joint_states.position.append(angle)
        pub.publish(lmm_joint_states)

    elif not is_inputs_zero:
        r_G_ee = r_G_ee + ee_increments
        x_i = [lmm_joint_angles['joint_m_1'], lmm_joint_angles['joint_m_2'], lmm_joint_angles['joint_m_3'], r_G_tb[0], r_G_tb[1]]
        red_resolver = redundancy_resolution.redundancy_resolver(inputs.w, inputs.l_man, inputs.r_l0_mb_p0, inputs.m_t, r_G_tb, r_G_ee, x_i, inputs.bounds)
        ik_lmm = inverse_kinematics.ik_lmm_solver(r_G_tb, inputs.eta_G_L0, r_G_ee, r_G_leg_tip, inputs.r_l0_si_p0, inputs.li, inputs.di, inputs.r_l0_mb_p0, inputs.l_man, inputs.d_man, inputs.phi, inputs.phi_m, inputs.gamma_r, inputs.gamma_l)
        man_joint_angles = ik_lmm.solve_manipulator()
        x_f = [man_joint_angles['joint_m_1'], man_joint_angles['joint_m_2'], man_joint_angles['joint_m_3'], r_G_tb[0], r_G_tb[1]]
        if red_resolver.ee_manipulability_constraint(x_f, inputs.m_t_man) >= 0:
            lmm_joint_angles = ik_lmm.solve_lmm()
            lmm_joint_states = JointState()
            lmm_joint_states.header.stamp = rospy.Time.now()
            for joint, angle in lmm_joint_angles.items():
                lmm_joint_states.name.append(joint)
                lmm_joint_states.position.append(angle)
            pub.publish(lmm_joint_states)
        else:
            # Redundancy Resolution
            x_f = red_resolver.resolve()

            # TB positions for crab angle calculation
            r_G_tb_i = r_G_tb
            r_G_tb_f = numpy.array([x_f[3], x_f[4], r_G_tb[2]])

            # Crab angle calculation
            theta_c = numpy.arctan2((r_G_tb_f[1]-r_G_tb_i[1]), (r_G_tb_f[0]-r_G_tb_i[0]))

            # TB time calculation
            tb_time = tb_time_calculation.tb_time_calculation(r_G_tb_i, r_G_tb_f, theta_c, inputs.vel_tb_max, inputs.T_stroke, inputs.T_in)

            # Gait Planning
            support_legs_list = [[2, 3, 4, 5], [1, 3, 4, 6], [1, 2, 5, 6]]      # Quadruped
            n_strokes = len(support_legs_list)

            # TB trajectory and Leg Tip Trajectory
            tb_pos_i = r_G_tb
            leg_tip_trajectory_local = {1:[], 2:[], 3:[], 4:[], 5:[], 6:[]}
            leg_tip_pos_f_local = {1:numpy.array([0,0,0]), 2:numpy.array([0,0,0]), 3:numpy.array([0,0,0]), 4:numpy.array([0,0,0]), 5:numpy.array([0,0,0]), 6:numpy.array([0,0,0])}
            for i in range(n_strokes):
                traj_r_G_tb_temp = tb_trajectory.tb_trajectory(theta_c, inputs.vel_tb_max, tb_pos_i, tb_time, inputs.T_in)
                traj_r_G_tb.extend(traj_r_G_tb_temp)
                n_traj_temp = len(traj_r_G_tb_temp)
                tb_pos_f = traj_r_G_tb_temp[n_traj_temp-1]
                stroke_length = numpy.hypot((tb_pos_f[0]-tb_pos_i[0]),(tb_pos_f[1]-tb_pos_i[1]))
                swing_length = n_strokes*stroke_length
                leg_tip_tr_local_temp = leg_tip_traj_local.leg_tip_traj_local(support_legs_list[i], swing_length, theta_c, inputs.T_stroke, inputs.T_in, inputs.Hmi1, inputs.del_h, inputs.hi3_dash, inputs.lt_time_ratio)
                leg_tip_trajectory_local_temp = leg_tip_tr_local_temp.calculate_trajectry()
                for j in range(6):
                    leg = j+1
                    for k in range(n_traj_temp):
                        leg_tip_trajectory_local_temp[leg][k] += leg_tip_pos_f_local[leg]
                    leg_tip_trajectory_local[leg].extend(leg_tip_trajectory_local_temp[leg])
                tb_pos_i = tb_pos_f
                n_traj = len(leg_tip_trajectory_local[1])
                for p in range(6):
                    leg = p+1
                    leg_tip_pos_f_local[leg] = leg_tip_trajectory_local[leg][n_traj-1]
            traj_r_G_leg_tip = leg_tip_traj_local_to_global.traj_local_to_global(r_G_tb, leg_tip_trajectory_local, tfBuffer)

            # Inverse Kinematics
            r_G_tb = traj_r_G_tb.pop(0)
            r_G_leg_tip = {}
            for i in range(6):
                leg = i+1
                r_G_leg_tip[leg] = traj_r_G_leg_tip[leg].pop(0)
            ik_lmm = inverse_kinematics.ik_lmm_solver(r_G_tb, inputs.eta_G_L0, r_G_ee, r_G_leg_tip, inputs.r_l0_si_p0, inputs.li, inputs.di, inputs.r_l0_mb_p0, inputs.l_man, inputs.d_man, inputs.phi, inputs.phi_m, inputs.gamma_r, inputs.gamma_l)
            lmm_joint_angles = ik_lmm.solve_lmm()
            lmm_joint_states = JointState()
            lmm_joint_states.header.stamp = rospy.Time.now()
            for joint, angle in lmm_joint_angles.items():
                lmm_joint_states.name.append(joint)
                lmm_joint_states.position.append(angle)
            pub.publish(lmm_joint_states)

    else:
        pass

    end_time = time.time()
    loop_time = end_time-start_time
    if loop_time > inputs.T_in:
        rospy.logwarn("Loop Time: " + str(loop_time))

    # Recording Data to CSV
    x_man = [lmm_joint_angles['joint_m_1'], lmm_joint_angles['joint_m_2'], lmm_joint_angles['joint_m_3'], r_G_tb[0], r_G_tb[1]]
    red_res_man = redundancy_resolution.redundancy_resolver(inputs.w, inputs.l_man, inputs.r_l0_mb_p0, inputs.m_t, r_G_tb, r_G_ee, x_man, inputs.bounds)
    manipulability = red_res_man.ee_manipulability_constraint(x_man, inputs.m_t_man) + inputs.m_t_man

    row = [time_instance*inputs.T_in, ee_increments[0], ee_increments[1], ee_increments[2], r_G_ee[0], r_G_ee[1], r_G_ee[2], r_G_tb[0], r_G_tb[1], r_G_tb[2], manipulability, stroke_length, lmm_joint_angles["joint_1_1"], lmm_joint_angles["joint_1_2"], lmm_joint_angles["joint_1_3"], lmm_joint_angles["joint_2_1"], lmm_joint_angles["joint_2_2"], lmm_joint_angles["joint_2_3"]]
    with open(data_file, 'a') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(row)

# Define the subscriber
rospy.Subscriber('lmm_incremental_inputs', Float32MultiArray, callback)

# Ready
rospy.loginfo('Im Ready')

# Keep spinning
rospy.spin()
