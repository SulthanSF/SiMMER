#!/usr/bin/env python

# Import required libraries
import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

# Initialize the node
rospy.init_node('tip_tf_broadcaster')

# Set the parameters
f_in = 10		# Publishing frequency in Hertz

# Define the publisher
tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=1)
rate = rospy.Rate(f_in)

# Define TF broadcaster class
class tf_broadcaster:
    def __init__(self, parent, child, trans, rot):
        t = TransformStamped()
        t.header.frame_id = parent
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = child
        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]
        t.transform.rotation.x = rot[0]
        t.transform.rotation.y = rot[1]
        t.transform.rotation.z = rot[2]
        t.transform.rotation.w = rot[3]
        self.tf = t
    def update_time_stamp(self):
        self.tf.header.stamp = rospy.Time.now()

# Define the transformations
leg1_tip = tf_broadcaster('Leg_1_Link_3', 'Leg_1_Tip', [-0.1, 0, 0], [0, 0, 0, 1])
leg2_tip = tf_broadcaster('Leg_2_Link_3', 'Leg_2_Tip', [0.1, 0, 0], [0, 0, 0, 1])
leg3_tip = tf_broadcaster('Leg_3_Link_3', 'Leg_3_Tip', [-0.1, 0, 0], [0, 0, 0, 1])
leg4_tip = tf_broadcaster('Leg_4_Link_3', 'Leg_4_Tip', [0.1, 0, 0], [0, 0, 0, 1])
leg5_tip = tf_broadcaster('Leg_5_Link_3', 'Leg_5_Tip', [-0.1, 0, 0], [0, 0, 0, 1])
leg6_tip = tf_broadcaster('Leg_6_Link_3', 'Leg_6_Tip', [0.1, 0, 0], [0, 0, 0, 1])
ee_tip = tf_broadcaster('Gripper', 'EE_Tip', [0, 0, 0.09], [0, 0, 0, 1])

# Publisher
while not rospy.is_shutdown():
    leg1_tip.update_time_stamp()
    leg2_tip.update_time_stamp()
    leg3_tip.update_time_stamp()
    leg4_tip.update_time_stamp()
    leg5_tip.update_time_stamp()
    leg6_tip.update_time_stamp()
    ee_tip.update_time_stamp()
    tfm = TFMessage([leg1_tip.tf, leg2_tip.tf, leg3_tip.tf, leg4_tip.tf, leg5_tip.tf, leg6_tip.tf, ee_tip.tf])
    tf_pub.publish(tfm)
    rate.sleep()
