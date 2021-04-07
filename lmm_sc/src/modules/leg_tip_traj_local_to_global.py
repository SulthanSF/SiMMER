#!/usr/bin/env python

# Import required libraries
import rospy
import tf2_ros
import numpy
import geometry_msgs.msg

def traj_local_to_global(tb_pos, leg_tip_traj_local, tfBuffer):
    n = numpy.shape(leg_tip_traj_local[1])[0]
    tb_pos_matrix = numpy.tile(tb_pos, (n,1))
    tf_tb_to_leg = {}
    leg_tip_traj_global = {}
    tf_leg_to_tb_matrix = {}
    tf_tb_to_leg[1] = tfBuffer.lookup_transform('Trunk_Body', 'Leg_1_Tip', rospy.Time())
    tf_tb_to_leg[2] = tfBuffer.lookup_transform('Trunk_Body', 'Leg_2_Tip', rospy.Time())
    tf_tb_to_leg[3] = tfBuffer.lookup_transform('Trunk_Body', 'Leg_3_Tip', rospy.Time())
    tf_tb_to_leg[4] = tfBuffer.lookup_transform('Trunk_Body', 'Leg_4_Tip', rospy.Time())
    tf_tb_to_leg[5] = tfBuffer.lookup_transform('Trunk_Body', 'Leg_5_Tip', rospy.Time())
    tf_tb_to_leg[6] = tfBuffer.lookup_transform('Trunk_Body', 'Leg_6_Tip', rospy.Time())
    for i in range(6):
        leg = i+1
        tf_leg_to_tb_matrix[leg] = numpy.tile(numpy.array([tf_tb_to_leg[leg].transform.translation.x, tf_tb_to_leg[leg].transform.translation.y, tf_tb_to_leg[leg].transform.translation.z]), (n,1))
        leg_tip_traj_global[leg] = tb_pos_matrix + tf_leg_to_tb_matrix[leg] + numpy.array(leg_tip_traj_local[leg])
        leg_tip_traj_global[leg] = leg_tip_traj_global[leg].tolist()
    return leg_tip_traj_global
