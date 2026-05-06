#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniFeedback
from collections import deque
import time
import matplotlib.pyplot as plt 

class HapticForceController:

    def __init__(self):

        # Initializing node
        rospy.init_node('haptic_force_controller', anonymous=True)

        # Publishers
        self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)

        # Subscribers
        rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)

        # For initialization parameter
        self.start_time = time.time()
        self.robot_force = np.zeros(3)
        self.robot_force_initial = np.zeros(3)
        self.joint_position_robot = np.zeros(6)
        self.robot_force_flag = True
        self.haptic_force = np.zeros(3)

        # For moving average filter
        self.haptic_window_size = 100
        self.force_window = deque(maxlen=self.haptic_window_size)

        # Shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        self.shutdown_flag = False

        # For plotting
        self.start_time = None
        self.time_stamps = []
        self.robot_force_plot = []
        self.haptic_force_plot = []

        self.magnitude_robot_force = []
        self.magnitude_haptic_force = []

    def update_list(self):
        if self.shutdown_flag:
            return
        current_time = rospy.get_time()
        if self.start_time is None:
            self.start_time = current_time
        self.time_stamps.append(current_time - self.start_time)
        #self.robot_force_plot.append(self.forcevector_conversion(self.joint_position_robot, self.robot_force)) #modifed later (robot_froce to robot_force_converted)
        self.robot_force_plot.append(self.robot_force)
        self.haptic_force_plot.append(self.haptic_force) 

        self.magnitude_robot_force.append(np.linalg.norm(self.robot_force))
        self.magnitude_haptic_force.append(np.linalg.norm(self.haptic_force))        

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

         # Z-axis of the sensor frame in base frame coordinates (3rd column of R)
        z_axis_sensor_base = R[:, 2]
        
        # Z-axis of the base frame in base frame coordinates
        z_axis_base = np.array([0, 0, 1])
        
        # Compute the normal vector to the plane formed by z-axis of sensor and z-axis of base
        normal_vector_base = np.cross(z_axis_base, z_axis_sensor_base)
        
        # Normalize the vector to get a unit vector
        unit_normal_vector_base = normal_vector_base / np.linalg.norm(normal_vector_base)
        
        # Convert the unit normal vector to the sensor frame
        unit_normal_vector_sensor = R.T @ unit_normal_vector_base

        '''for calibration'''
        dot_product = np.dot(z_axis_base, z_axis_sensor_base)

        # Compute the angle using arctan2
        angle = np.arctan2(np.linalg.norm(normal_vector_base), dot_product)

        correction_factor = 1.34*sin(angle)*unit_normal_vector_sensor

        print("correction factor", correction_factor)

        print("robot_force_before:",robot_force)
        #testing below
        g_base = np.array([0, 0, -3.6335])
        g_sensor = np.dot(R.T, g_base)
        print("g_sensor:",g_sensor)
        F_offset = np.array([0, 0, -3.834 ])
        robot_force = robot_force - g_sensor - F_offset - correction_factor
        print("robot_force_after:",robot_force)
        #testing till

        '''for calibration'''

        return np.dot(R, robot_force)

    def joint_callback_robot(self, msg: JointState):
        self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])

    def robot_force_callback(self, msg: WrenchStamped):
        if self.robot_force_flag:
            self.robot_force_initial = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
            if time.time() - self.start_time < 1:
                rospy.loginfo("booting")
                return
            self.robot_force_flag = False
        robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])# - self.robot_force_initial
        self.force_window.append(robot_force)
        self.robot_force = np.mean(self.force_window, axis=0)
        print("magnitude:",np.linalg.norm(self.robot_force))
        #print("initial force:",self.robot_force_initial)

    def haptic_force_callback(self, event):
        if self.shutdown_flag:
            return
        force_pub_msg = OmniFeedback()
        robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z = robot_force
        #self.force_pub.publish(force_pub_msg)
        self.haptic_force = np.array([force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z])
        self.update_list()

    def plot_data(self):

        times = np.array(self.time_stamps)
        robot_force_plot = np.array(self.robot_force_plot)
        haptic_force_plot = np.array(self.haptic_force_plot)

        magnitude_robot_force = np.array(self.magnitude_robot_force)
        magnitude_haptic_force = np.array(self.magnitude_haptic_force)

        fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
        axs[0, 0].plot(times, robot_force_plot[:, 0], label='robot force')
        axs[0, 0].plot(times, haptic_force_plot[:, 0], label='haptic force')
        axs[0, 0].set_title('Force in x')
        axs[0, 0].set_ylabel('Newtons')
        axs[0, 0].set_xlabel('Seconds')
        axs[0, 0].grid(True)
        axs[0, 0].legend()

        axs[1, 0].plot(times, robot_force_plot[:, 1], label='robot force')
        axs[1, 0].plot(times, haptic_force_plot[:, 1], label='haptic force')
        axs[1, 0].set_title('Force in y')
        axs[1, 0].set_ylabel('Newtons')
        axs[1, 0].set_xlabel('Seconds')
        axs[1, 0].grid(True)
        axs[1, 0].legend()

        axs[2, 0].plot(times, robot_force_plot[:, 2], label='robot force')
        axs[2, 0].plot(times, haptic_force_plot[:, 2], label='haptic force')
        axs[2, 0].set_title('Force in z')
        axs[2, 0].set_ylabel('Newtons')
        axs[2, 0].set_xlabel('Seconds')
        axs[2, 0].grid(True)
        axs[2, 0].legend()

        # axs[0, 1].plot(times, robot_force_plot[:, 0] - haptic_force_plot[:, 0])
        # axs[0, 1].set_title('Error in x')
        # axs[0, 1].set_ylabel('Newtons')
        # axs[0, 1].set_xlabel('Seconds')
        # axs[0, 1].grid(True)

        axs[0, 1].plot(times, magnitude_robot_force)
        axs[0, 1].set_title('magnitude_robot_force')
        axs[0, 1].set_ylabel('Newtons')
        axs[0, 1].set_xlabel('Seconds')
        axs[0, 1].grid(True)

        # axs[1, 1].plot(times, robot_force_plot[:, 1] - haptic_force_plot[:, 1])
        # axs[1, 1].set_title('Error in y')
        # axs[1, 1].set_ylabel('Newtons')
        # axs[1, 1].set_xlabel('Seconds')
        # axs[1, 1].grid(True)

        axs[1, 1].plot(times, magnitude_haptic_force)
        axs[1, 1].set_title('magnitude_haptic_force')
        axs[1, 1].set_ylabel('Newtons')
        axs[1, 1].set_xlabel('Seconds')
        axs[1, 1].grid(True)

        axs[2, 1].plot(times, robot_force_plot[:, 2] - haptic_force_plot[:, 2])
        axs[2, 1].set_title('Error in z')
        axs[2, 1].set_ylabel('Newtons')
        axs[2, 1].set_xlabel('Seconds')
        axs[2, 1].grid(True)

        plt.tight_layout()
        plt.show()

    def main_loop(self):
        rate = 1000
        rospy.Timer(rospy.Duration(1.0 / rate), self.haptic_force_callback)
        rospy.spin()

    def shutdown_hook(self):
        self.shutdown_flag = True
        zero_force_msg = OmniFeedback()
        zero_force_msg.force.x, zero_force_msg.force.y, zero_force_msg.force.z = (0, 0, 0)
        rospy.loginfo("Sending zero force to haptic device.")
        for _ in range(10):
            self.force_pub.publish(zero_force_msg)
            time.sleep(0.1)
        rospy.loginfo("Shutting down, sent zero force to haptic device.")
        self.plot_data()

if __name__ == "__main__":
    try:
        controller = HapticForceController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass