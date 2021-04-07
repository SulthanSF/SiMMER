#!/usr/bin/env python

#ROS node for controlling servo motors using I2CPWM board

#Import required libraries
import rospy
import rospkg
import yaml
from math import pi
from sensor_msgs.msg import JointState
from i2cpwm_board.msg import Servo
from i2cpwm_board.msg import ServoArray

#Import config file
rospack = rospkg.RosPack()
package_path = rospack.get_path('lmm_sc')
joints_config = file(package_path + '/config/joints.yaml','r')
lmm_joints = yaml.load(joints_config)['lmm_joints']

#Initialize the node
rospy.init_node('motor_controller')

#Define the publisher
pub = rospy.Publisher('servos_absolute', ServoArray, queue_size = 10)

#Define callback to publish to the board
def callback(joint_data):
    servos = []
    for joint in range(len(joint_data.name)):
        servo_data = Servo()
        joint_mean_angle = (lmm_joints[joint_data.name[joint]]['max_angle']+lmm_joints[joint_data.name[joint]]['min_angle'])/2
        joint_angle_range = lmm_joints[joint_data.name[joint]]['max_angle']-lmm_joints[joint_data.name[joint]]['min_angle']
        joint_mean_input = (lmm_joints[joint_data.name[joint]]['max_input']+lmm_joints[joint_data.name[joint]]['min_input'])/2
        joint_input_range = lmm_joints[joint_data.name[joint]]['max_input']-lmm_joints[joint_data.name[joint]]['min_input']
        joint_axis = lmm_joints[joint_data.name[joint]]['axis']
        servo_value = joint_mean_input + joint_axis * (joint_input_range/joint_angle_range) * (joint_data.position[joint]-joint_mean_angle)
        servo_data.servo = lmm_joints[joint_data.name[joint]]['servo']
        servo_data.value = servo_value
        servos.append(servo_data)
    servos_data = ServoArray()
    servos_data.servos = servos
    pub.publish(servos_data)

#Define the subscriber
rospy.Subscriber('lmm_joint_states', JointState, callback)

#Keep spinning
rospy.spin()
