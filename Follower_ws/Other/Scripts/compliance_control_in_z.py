#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniFeedback
from collections import deque
import time
import matplotlib.pyplot as plt 
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64MultiArray

class HapticForceController:

    def __init__(self):

        # Initializing node
        rospy.init_node('haptic_force_controller', anonymous=True)

        # Publishers
        self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)

        # Subscribers
        rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)

        # For initialization parameter
        self.velocity_z = 0.0
        self.robot_force = np.zeros(3)
        self.robot_force_initial = np.zeros(3)
        self.joint_position_robot = np.zeros(6)
        self.current_ee_pose = np.zeros((6, 1))
        self.joint_position_robot = np.zeros(6)
        self.controller_flag = False

        # For moving average filter
        self.haptic_window_size = 100
        self.force_window = deque(maxlen=self.haptic_window_size)

        # Shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        self.shutdown_flag = False

        # For plotting
        self.start_time = time.time() 
        self.time_stamps = []
        self.robot_force_plot = []
        self.threshold = 2.0

        # For unwrapping
        self.robot_previous_angles = None

    def update_list(self):
        if self.shutdown_flag:
            return
        current_time = rospy.get_time()
        if self.start_time is None:
            self.start_time = current_time
        self.time_stamps.append(current_time - self.start_time)
        self.robot_force_plot.append(self.forcevector_conversion(self.joint_position_robot, self.robot_force))

    @staticmethod
    def Jointspace2GeometricJacobian(joint_position_robot):
        t1, t2, t3, t4, t5, t6 = joint_position_robot
        cos, sin = np.cos, np.sin
        J_geometrical = np.array([
            [
                (2621*cos(t1))/20000 + (2371*cos(t1)*cos(t5))/10000 + (4871*cos(t2)*sin(t1))/20000 - 
                (533*sin(t1)*sin(t2)*sin(t3))/2500 + (2371*cos(t2 + t3 + t4)*sin(t1)*sin(t5))/10000 - 
                (1707*cos(t2 + t3)*sin(t1)*sin(t4))/20000 - (1707*sin(t2 + t3)*cos(t4)*sin(t1))/20000 + 
                (533*cos(t2)*cos(t3)*sin(t1))/2500, 
                cos(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
                sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
                (1707*cos(t2 + t3)*cos(t4))/20000), 
                cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
                (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
                cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
                (2371*cos(t1)*cos(t2)*cos(t5)*sin(t3)*sin(t4))/10000 - 
                (2371*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5))/10000 - (2371*sin(t1)*sin(t5))/10000 + 
                (2371*cos(t1)*cos(t3)*cos(t5)*sin(t2)*sin(t4))/10000 + 
                (2371*cos(t1)*cos(t4)*cos(t5)*sin(t2)*sin(t3))/10000, 
                0
            ],
            [
                (2621*sin(t1))/20000 - (4871*cos(t1)*cos(t2))/20000 + (2371*cos(t5)*sin(t1))/10000 + 
                (533*cos(t1)*sin(t2)*sin(t3))/2500 - (2371*cos(t2 + t3 + t4)*cos(t1)*sin(t5))/10000 + 
                (1707*cos(t2 + t3)*cos(t1)*sin(t4))/20000 + (1707*sin(t2 + t3)*cos(t1)*cos(t4))/20000 - 
                (533*cos(t1)*cos(t2)*cos(t3))/2500, 
                sin(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
                sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
                (1707*cos(t2 + t3)*cos(t4))/20000), 
                sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
                (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
                sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
                (2371*cos(t1)*sin(t5))/10000 - (2371*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1))/10000 + 
                (2371*cos(t2)*cos(t5)*sin(t1)*sin(t3)*sin(t4))/10000 + 
                (2371*cos(t3)*cos(t5)*sin(t1)*sin(t2)*sin(t4))/10000 + 
                (2371*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t3))/10000, 
                0
            ],
            [
                0, 
                (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
                (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500 - (4871*cos(t2))/20000, 
                (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
                (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500, 
                (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
                (2371*sin(t2 + t3 + t4 + t5))/20000, 
                - (2371*sin(t2 + t3 + t4 - t5))/20000 - (2371*sin(t2 + t3 + t4 + t5))/20000, 
                0
            ],
            [
                0, 
                sin(t1), 
                sin(t1), 
                sin(t1), 
                sin(t2 + t3 + t4)*cos(t1), 
                cos(t5)*sin(t1) - cos(t2 + t3 + t4)*cos(t1)*sin(t5)
            ],
            [
                0, 
                -cos(t1), 
                -cos(t1), 
                -cos(t1), 
                sin(t2 + t3 + t4)*sin(t1), 
                - cos(t1)*cos(t5) - cos(t2 + t3 + t4)*sin(t1)*sin(t5)
            ],
            [
                1, 
                0, 
                0, 
                0, 
                -cos(t2 + t3 + t4), 
                -sin(t2 + t3 + t4)*sin(t5)
            ]
        ])
        lamda = 0.001
        return J_geometrical + lamda * np.eye(6)

    def rpy2joint_space_vel(self):

        alpha, beta, gamma = self.current_ee_pose[3:6, 0]
        cos, sin = np.cos, np.sin
        
        J_geo2ana = np.array([[1, 0, 0,                    0,           0, 0],
                              [0, 1, 0,                    0,           0, 0],
                              [0, 0, 1,                    0,           0, 0],
                              [0, 0, 0, cos(beta)*cos(gamma), -sin(gamma), 0],
                              [0, 0, 0, cos(beta)*sin(gamma),  cos(gamma), 0],
                              [0, 0, 0,           -sin(beta),           0, 1]])

        J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)

        gain = 1.0
        k = gain * np.eye(6)
 
        ee_velocity = np.zeros((6, 1))
        ee_velocity[2, 0] = self.velocity_z 
        joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (ee_velocity)
        
        return joint_space_velocity.flatten()
    
    @staticmethod
    def unwrap_angle(angle, previous_angle):
        if previous_angle is None:
            return angle
        delta = angle - previous_angle
        delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
        delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
        delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
        return previous_angle + delta
    
    def pose_and_rotation(self,joint_position_robot):
        q1, q2, q3, q4, q5, q6 = joint_position_robot

        cos = np.cos
        sin = np.sin
        a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
        # Calculate each component of the position
        x = (a1 * sin(q1)) / a7 - (a2 * cos(q1) * cos(q2)) / a7 + (a3 * cos(q5) * sin(q1)) / a9 - \
            (a3 * cos(q2 + q3 + q4) * cos(q1) * sin(q5)) / a9 + (a4 * cos(q2 + q3) * cos(q1) * sin(q4)) / a7 + \
            (a4 * sin(q2 + q3) * cos(q1) * cos(q4)) / a7 - (a5 * cos(q1) * cos(q2) * cos(q3)) / a8 + \
            (a5 * cos(q1) * sin(q2) * sin(q3)) / a8

        y = (a5 * sin(q1) * sin(q2) * sin(q3)) / a8 - (a3 * cos(q1) * cos(q5)) / a9 - \
            (a2 * cos(q2) * sin(q1)) / a7 - (a1 * cos(q1)) / a7 - (a3 * cos(q2 + q3 + q4) * sin(q1) * sin(q5)) / a9 + \
            (a4 * cos(q2 + q3) * sin(q1) * sin(q4)) / a7 + (a4 * sin(q2 + q3) * cos(q4) * sin(q1)) / a7 - \
            (a5 * cos(q2) * cos(q3) * sin(q1)) / a8

        z = (a4 * sin(q2 + q3) * sin(q4)) / a7 - (a2 * sin(q2)) / a7 - sin(q5) * ((a3 * cos(q2 + q3) * sin(q4)) / a9 + \
            (a3 * sin(q2 + q3) * cos(q4)) / a9) - (a4 * cos(q2 + q3) * cos(q4)) / a7 - (a5 * sin(q2 + q3)) / a8 + a6 / a7

        # Compute each element of the rotation matrix
        r11 = cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6)
        r12 = -sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6)
        r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        
        r21 = -cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6)
        r22 = sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1)
        r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        
        r31 = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)
        r32 = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)
        r33 = -sin(q2 + q3 + q4)*sin(q5)
        
        R_matrix = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
       
        rotation = R.from_matrix(R_matrix)
        euler_angles = rotation.as_euler('xyz', degrees=False)
        euler_angles = np.array(euler_angles)
        if self.robot_previous_angles is not None:
            euler_angles = self.unwrap_angle(euler_angles, self.robot_previous_angles)
        self.robot_previous_angles = euler_angles

        return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])

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
        return (np.dot(R, robot_force))

    def joint_callback_robot(self, msg: JointState):
        self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
        self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)

    def robot_force_callback(self, msg: WrenchStamped):
        if time.time() - self.start_time < 1:
            rospy.loginfo("booting")
        self.controller_flag = True
        robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.force_window.append(robot_force)
        self.robot_force = np.mean(self.force_window, axis=0)

    def compliance_control(self, event):
        if self.shutdown_flag:
            return
        robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        self.control_law(robot_force)
        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = self.rpy2joint_space_vel()
        self.velocity_pub.publish(velocity_pub_msg)
        self.update_list()

    def control_law(self, robot_force):
        force_x, force_y, force_z = robot_force
        deadband = self.threshold      
        k = 0.1             
        max_velocity = 0.2  
        self.velocity_z = 0.0
        if self.controller_flag:
            if force_z > -deadband: 
                self.velocity_z = (force_z - deadband) * k
            elif force_z < -deadband:  
                self.velocity_z = (force_z + deadband) * k 
            self.velocity_z = np.clip(self.velocity_z, -max_velocity, max_velocity)

    def main_loop(self):
        rate = 500
        rospy.Timer(rospy.Duration(1.0 / rate), self.compliance_control)
        rospy.spin()

    def plot_data_z(self):
        times = np.array(self.time_stamps)
        robot_force_plot = np.array(self.robot_force_plot)
        threshold_plot = np.full_like(times, self.threshold)   
        plt.figure(figsize=(10, 5))
        plt.plot(times, robot_force_plot[:, 2], label='Measured force')
        plt.plot(times, threshold_plot, 'r--', label='Desired force') 
        plt.title('Force in Z-direction')
        plt.ylabel('Force (N)')
        plt.xlabel('Time (s)')
        plt.grid(True)
        plt.legend()
        max_force = np.max(robot_force_plot[:, 2])
        min_force = np.min(robot_force_plot[:, 2])
        plt.annotate(f'Max: {max_force:.2f}N', xy=(0.02, 0.95), xycoords='axes fraction')
        plt.annotate(f'Min: {min_force:.2f}N', xy=(0.02, 0.88), xycoords='axes fraction')
        plt.tight_layout()
        plt.show()

    def shutdown_hook(self):
        self.shutdown_flag = True
        rospy.loginfo("Sending zero force to haptic device.")
        for _ in range(10):
            time.sleep(0.1)
        rospy.loginfo("Shutting down, sent zero force to haptic device.")
        self.plot_data_z()

if __name__ == "__main__":
    try:
        controller = HapticForceController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass