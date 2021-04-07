#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

def joint_controller_pub():
    pub_joint_state = {}
    pub_joint_state['joint_1_1'] = rospy.Publisher('/lmm/joint_1_1_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_1_2'] = rospy.Publisher('/lmm/joint_1_2_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_1_3'] = rospy.Publisher('/lmm/joint_1_3_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_2_1'] = rospy.Publisher('/lmm/joint_2_1_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_2_2'] = rospy.Publisher('/lmm/joint_2_2_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_2_3'] = rospy.Publisher('/lmm/joint_2_3_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_3_1'] = rospy.Publisher('/lmm/joint_3_1_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_3_2'] = rospy.Publisher('/lmm/joint_3_2_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_3_3'] = rospy.Publisher('/lmm/joint_3_3_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_4_1'] = rospy.Publisher('/lmm/joint_4_1_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_4_2'] = rospy.Publisher('/lmm/joint_4_2_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_4_3'] = rospy.Publisher('/lmm/joint_4_3_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_5_1'] = rospy.Publisher('/lmm/joint_5_1_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_5_2'] = rospy.Publisher('/lmm/joint_5_2_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_5_3'] = rospy.Publisher('/lmm/joint_5_3_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_6_1'] = rospy.Publisher('/lmm/joint_6_1_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_6_2'] = rospy.Publisher('/lmm/joint_6_2_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_6_3'] = rospy.Publisher('/lmm/joint_6_3_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_m_1'] = rospy.Publisher('/lmm/joint_m_1_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_m_2'] = rospy.Publisher('/lmm/joint_m_2_position_controller/command', Float64, queue_size=10)
    pub_joint_state['joint_m_3'] = rospy.Publisher('/lmm/joint_m_3_position_controller/command', Float64, queue_size=10)
    
    rospy.init_node('joint_controller_pub', anonymous=True)
    
    def callback(joint_data):
        for joint in range(len(joint_data.name)):
            pub_joint_state[joint_data.name[joint]].publish(joint_data.position[joint])
        
    # Define the subscriber
    rospy.Subscriber('lmm_joint_states', JointState, callback)

    rospy.spin()   
    
if __name__ == '__main__':
    try:
        joint_controller_pub()
    except rospy.ROSInterruptException:
        pass
