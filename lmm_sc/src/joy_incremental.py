#!/usr/bin/env python

# ROS node for converting joystick inputs to position increments

# Import required libraries
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

# Initialize the node
rospy.init_node('joy_incremental')

# Set the parameters
multipliers = [-5.0, 5.0, 5.0]
divider = 4.0
f_in = 10		# Publishing frequency in Hertz

# Initialize the input vectors
joy_input = [0, 0, 0]
lmm_input = [0, 0, 0]

# Define the publisher
pub = rospy.Publisher('lmm_incremental_inputs', Float32MultiArray, queue_size = 10)
rate = rospy.Rate(f_in)

# Define callback to update the input vector everytime a msg is received
def callback(data):
	global joy_input
	joy_input[0] = data.axes[3]
	joy_input[1] = data.axes[4]
	joy_input[2] = data.axes[1]

# Define the subscriber
rospy.Subscriber('joy', Joy, callback)

# Publisher
while not rospy.is_shutdown():
	lmm_input[0] = (1/divider)*round(multipliers[0]*joy_input[0])*10**-3
	lmm_input[1] = (1/divider)*round(multipliers[1]*joy_input[1])*10**-3
	lmm_input[2] = (1/divider)*round(multipliers[2]*joy_input[2])*10**-3
	lmm_input_data = Float32MultiArray()
	lmm_input_data.data = lmm_input
	pub.publish(lmm_input_data)
	rate.sleep()
