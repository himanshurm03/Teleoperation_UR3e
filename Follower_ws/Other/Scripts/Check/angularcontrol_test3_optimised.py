#!/usr/bin/env python3

import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import math

class RobotEndEffectorController:
    def __init__(self):
        self.ee_start = True
        self.robot_offset = np.array([-0.24355, 0, 0, 0, 0, 0, 0])
        self.current_ee_pose = np.zeros((6, 1))
        self.initial_ee_pose = np.zeros((6, 1))
        self.haptic_stylus_position = np.zeros(3)
        self.h_offset_array = np.zeros(7)
        self.flag_pos_offset = True
        self.haptic_current_linear_pose = np.zeros((3, 1))

        self.joint_position_context = np.zeros(6)

        self.th_offsets = np.zeros(6)
        self.flag_offset = True
        self.haptic_current_angular_pose = np.zeros((3, 1))

        rospy.init_node('robot_end_effector_controller', anonymous=True)

        self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)

        rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        rospy.Subscriber('/joint_states', JointState, self.haptic_offset)

    def quaternion_to_euler(self, quat):
        rotation = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
        return rotation.as_euler('xyz', degrees=False)

    def dead_band_th(self, theta_array):
        return [0.0 if abs(j) < 0.0014 else j for j in theta_array]

    def Q(self, theta_q):
        def Qx(th):
            return np.array([[1, 0, 0], [0, math.cos(th), -math.sin(th)], [0, math.sin(th), math.cos(th)]])
        
        def Qy(th):
            return np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]])
        
        def Qz(th):
            return np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1]])

        q1, q2, q3, q4, q5, q6 = Qz(theta_q[0]), Qx(theta_q[1]), Qx(theta_q[2]), Qz(theta_q[3]), Qx(theta_q[4]), Qy(theta_q[5])
        return q1 @ q2 @ q3 @ q4 @ q5 @ q6

    def tf_callback(self, msg: TFMessage):
        X_pos_ee = msg.transforms[0].transform.translation.x
        Y_pos_ee = msg.transforms[0].transform.translation.y
        Z_pos_ee = msg.transforms[0].transform.translation.z

        W_ori_ee = msg.transforms[0].transform.rotation.w
        X_ori_ee = msg.transforms[0].transform.rotation.x
        Y_ori_ee = msg.transforms[0].transform.rotation.y
        Z_ori_ee = msg.transforms[0].transform.rotation.z

        quat_from_robot = [W_ori_ee, X_ori_ee, Y_ori_ee, Z_ori_ee]
        euler_from_robot = self.quaternion_to_euler(quat_from_robot)

        current_ee_pose = np.array([[X_pos_ee], [Y_pos_ee], [Z_pos_ee], [euler_from_robot[0]], [euler_from_robot[1]], [euler_from_robot[2]]])

        if self.ee_start:
            self.initial_ee_pose = current_ee_pose
            self.ee_start = False
            #rospy.loginfo(f"End_effector_start: {self.end_effector_start}")

        self.current_ee_pose = current_ee_pose

    def haptic_callback(self, msg: PoseStamped):
        if self.flag_pos_offset:
            self.h_offset_array = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                                            msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
            self.flag_pos_offset = False

        self.haptic_stylus_position = np.array([msg.pose.position.x - self.h_offset_array[0],
                                                msg.pose.position.y - self.h_offset_array[1],
                                                msg.pose.position.z - self.h_offset_array[2]])

        self.haptic_current_linear_pose = self.haptic_stylus_position.reshape(3, 1)

    def joint_callback_robot(self, msg: JointState):
        self.joint_position_context = msg.position

    def haptic_offset(self, msg: JointState):
        if self.flag_offset:
            self.th_offsets = np.array([-msg.position[0], msg.position[1], -msg.position[2], msg.position[3], msg.position[4], msg.position[5]])
            self.flag_offset = False

        th_array = np.array([-msg.position[0] - self.th_offsets[0],
                             msg.position[1] - self.th_offsets[1],
                             msg.position[2] + self.th_offsets[2],
                             (-msg.position[3] + self.th_offsets[3]) if msg.position[3] > 0 else (msg.position[3] + self.th_offsets[3]),
                             msg.position[4] - self.th_offsets[4] if msg.position[4] != 0 else 0.0,
                             msg.position[5] - self.th_offsets[5] if msg.position[5] != 0 else 0.0])

        th_array = self.dead_band_th(th_array)

        q = self.Q(th_array)
        q0_h = math.sqrt(1 + q[0, 0] + q[1, 1] + q[2, 2]) / 2
        if q0_h != 0:
            q1_h = (q[2, 1] - q[1, 2]) / (4 * q0_h)
            q2_h = (q[0, 2] - q[2, 0]) / (4 * q0_h)
            q3_h = (q[1, 0] - q[0, 1]) / (4 * q0_h)
        else:
            q1_h = math.sqrt((q[0, 0] + 1)/2)
            q2_h = math.sqrt((q[1, 1] + 1)/2)
            q3_h = math.sqrt((q[2, 2] + 1)/2)

        haptic_quat = [q0_h, q1_h, q2_h, q3_h]
        self.haptic_current_angular_pose = np.array(self.quaternion_to_euler(haptic_quat)).reshape(3, 1)

    def desired_traj(self):
        haptic_current_pose = np.vstack((self.haptic_current_linear_pose, self.haptic_current_angular_pose))
        return self.initial_ee_pose + haptic_current_pose

    def Jointspace2GeometricJacobian(self):
        a_end = np.array([0, 0, 0.145, 1])
        epsilon = np.array([1, 1, 1, 1, 1, 1])

        dh = np.array([[np.pi/2, 0, 0.15185, self.joint_position_context[0]],
                       [0, -0.24355, 0, self.joint_position_context[1]],
                       [0, -0.2132, 0, self.joint_position_context[2]],
                       [np.pi/2, 0, 0.13105, self.joint_position_context[3]],
                       [-np.pi/2, 0, 0.08535, self.joint_position_context[4]],
                       [0, 0, 0.0921, self.joint_position_context[5]]])

        T = np.eye(4)
        for n in range(dh.shape[0]):
            alp, a, d, theta = dh[n]
            t = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alp), np.sin(theta) * np.sin(alp), a * np.cos(theta)],
                          [np.sin(theta), np.cos(theta) * np.cos(alp), -np.cos(theta) * np.sin(alp), a * np.sin(theta)],
                          [0, np.sin(alp), np.cos(alp), d],
                          [0, 0, 0, 1]])
            T = T @ t

        P = (T @ a_end)[:3]

        T1 = np.eye(4)
        Jv = np.zeros((3, dh.shape[0]))
        Jw = np.zeros((3, dh.shape[0]))

        for n in range(dh.shape[0]):
            alp, a, d, theta = dh[n]
            t = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alp), np.sin(theta) * np.sin(alp), a * np.cos(theta)],
                          [np.sin(theta), np.cos(theta) * np.cos(alp), -np.cos(theta) * np.sin(alp), a * np.sin(theta)],
                          [0, np.sin(alp), np.cos(alp), d],
                          [0, 0, 0, 1]])
            T1 = T1 @ t

            Z = T1[:3, 2]
            oo = T1[:3, 3]
            Jv[:, n] = epsilon[n] * np.cross(Z, P - oo) + (1 - epsilon[n]) * Z
            Jw[:, n] = epsilon[n] * Z

        lamda = 0.001
        J_geometrical = np.vstack((Jv, Jw)) + lamda * np.eye(6)
        return J_geometrical

    def rpy2joint_space_vel(self):
        alpha, beta, gamma = 0, 0, 0
        A = np.array([[np.sin(beta), 0, 1],
                      [-np.cos(beta) * np.sin(gamma), np.cos(gamma), 0],
                      [np.cos(beta) * np.cos(gamma), np.sin(gamma), 0]])

        J_geo2ana = np.block([[np.eye(3), np.zeros((3, 3))],
                              [np.zeros((3, 3)), np.linalg.inv(A)]])
        
        J_analytical = J_geo2ana @ self.Jointspace2GeometricJacobian()

        k = np.eye(6)
        joint_space_velocity = np.linalg.inv(J_analytical) @ k @ (self.desired_traj() - self.current_ee_pose)
        return joint_space_velocity.flatten()

    def main_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            desired_ee_trajectory = self.desired_traj()

            velocity_pub_msg = Float64MultiArray()
            velocity_pub_msg.data = self.rpy2joint_space_vel()
            self.velocity_pub.publish(velocity_pub_msg)

            rate.sleep()

if __name__ == "__main__":
    try:
        controller = RobotEndEffectorController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass