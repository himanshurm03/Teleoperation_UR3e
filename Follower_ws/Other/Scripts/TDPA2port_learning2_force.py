#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from collections import deque
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray

class TDPAslave:

    def __init__(self):

        # Initialize node
        rospy.init_node('Slave_force', anonymous=True)

        # For initialization parameter
        self.joint_position_robot = np.zeros(6)
        self.robot_force = np.zeros(3)
        self.start_time = None

        # For unwrapping
        self.robot_previous_angles = None

        # For moving average filter
        self.force_window_size = 100
        self.force_window = deque(maxlen=self.force_window_size)

        # Publisher
        self.force_pub = rospy.Publisher('force_from_slave_to_master', Float64MultiArray, queue_size=1)
        self.force_pub2 = rospy.Publisher('corrected_force', Float64MultiArray, queue_size=1)

        # Subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)
        
    def joint_callback_robot(self, msg: JointState):
        self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
        
    def main_loop(self):
        rospy.spin()

    def robot_force_callback(self, msg: WrenchStamped):
        robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.force_window.append(robot_force)
        self.robot_force = np.mean(self.force_window, axis=0)

        force_pub_msg = Float64MultiArray()
        robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        force_pub_msg.data = robot_force.tolist() 
        #print(f"robot_force: {robot_force[2]:.3f}")
        self.force_pub.publish(force_pub_msg)
        self.force_pub2.publish(force_pub_msg)

    @staticmethod
    def forcevector_conversion(joint_position_robot, robot_force):
        q1, q2, q3, q4, q5, q6 = joint_position_robot
        cos = np.cos
        sin = np.sin
        r11 = -(cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6))
        r12 = -(-sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6))
        r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        r21 = -(-cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6))
        r22 = -(sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1))
        r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        r31 = -(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6))
        r32 = -(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6))
        r33 = -sin(q2 + q3 + q4)*sin(q5)
        R = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])

        z_axis_sensor_base = R[:, 2]
        z_axis_base = np.array([0, 0, 1])
        cross_product = np.cross(z_axis_base, z_axis_sensor_base)
        dot_product = np.dot(z_axis_base, z_axis_sensor_base)
        magnitude_of_cross_product_vector = np.linalg.norm(cross_product)
        unit_cross_product_base = cross_product / magnitude_of_cross_product_vector
        unit_cross_product_sensor = R.T @ unit_cross_product_base
        angle = np.arctan2(magnitude_of_cross_product_vector, dot_product)
        correction_factor = (1.34+0.15)*sin(angle)*unit_cross_product_sensor
        g_base = np.array([0, 0, -3.6335])
        g_sensor = np.dot(R.T, g_base)
        F_offset = np.array([0, 0, -3.834+0.51-0.4+0.02+0.04])
        robot_force = robot_force - g_sensor - F_offset - correction_factor

        '''not in the paper, just for compensating the inaccuracies from f/t sensor'''
        # robot_force = np.array([0.0, 0.0, 0.0])
        
        # if abs(robot_force[2])<0.01:
        #     robot_force[2]=0
        # robot_force = np.dot(R, robot_force)
        # return robot_force
        '''here the extra modification ends'''

        return (np.dot(R, robot_force))
    
if __name__ == "__main__":
    try:
        node = TDPAslave()
        node.main_loop()
    except rospy.ROSInterruptException:
        pass