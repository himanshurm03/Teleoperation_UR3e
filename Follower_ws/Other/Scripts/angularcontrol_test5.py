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

        rospy.init_node('robot_end_effector_controller', anonymous=True)

        # Publisher that publish joint angle velocity to the manipulator
        self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)

        # Provides end effector pose of manipulator in task space
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)

        # Provides end effector pose of haptic device in task space 
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

        # Provides joint angles of the manipulator
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)

        # Provides joint angles of haptic device
        rospy.Subscriber('/phantom/phantom/joint_states', JointState, self.haptic_offset)

        self.ee_start = True # to set initial offset to the end effector
        self.haptic_start_1 = True # to set initial offset to the stylus for /phantom/phantom/pose
        self.haptic_start_2 = True # to set initial offset to the stylus for /phantom/phantom/joint_states
        self.initial_ee_pose = np.zeros((6, 1))
        self.current_ee_pose = np.zeros((6, 1))
        self.haptic_current_linear_pose = np.zeros((3, 1))
        self.joint_position_robot = np.zeros((6, 1))
        self.haptic_current_angular_pose = np.zeros((3, 1))

    def quaternion_to_euler(quat):

        """ 
        Convert a quaternion into Euler angles (roll, pitch, yaw).

        Parameters:
        quat (list or tuple): A quaternion in the format [w, x, y, z].

        Returns:
        euler (numpy.ndarray): Euler angles in radians, in the order (roll, pitch, yaw).
        """

        # Create a Rotation object from the quaternion
        rotation = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
        
        # Convert the rotation object to Euler angles in the 'zyx' sequence
        euler = rotation.as_euler('zyx', degrees=False)
        
        return euler

    def Q(self, theta_haptic):
    
        """ 
        Calculates rotation matrix of stylus wrt to the base 

        Parameters:
        theta_haptic (list or tuple): joint state of haptic device.

        Returns:
        (numpy.ndarray): 3x3 rotation matrix that which represent stylus frame wrt base frame.
        """

        # Rotation about x-axis
        def Qx(th):
            return np.array([[1, 0, 0], [0, math.cos(th), -math.sin(th)], [0, math.sin(th), math.cos(th)]])
        
        # Rotation about y-axis
        def Qy(th):
            return np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]])
        
        # Rotation about z-axis
        def Qz(th):
            return np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1]])

        # Substituting values
        q1, q2, q3, q4, q5, q6 = Qz(theta_haptic[0]), Qx(theta_haptic[1]), Qx(theta_haptic[2]), Qz(theta_haptic[3]), Qx(theta_haptic[4]), Qy(theta_haptic[5])

        # Returning Rotation matrix which relates stylus frame to base frame
        return q1 @ q2 @ q3 @ q4 @ q5 @ q6
    
    def desired_traj(self):

        """ 
        Calculates desired pose for the end effector.

        Parameters:
        JointState: 1x6 list containing joint angles of haptic device.

        Returns:
        self.haptic_current_angular_pose: 3x1 list containing joint angles of haptic device.
        """
        # Concatenating haptic linear and angular pose
        haptic_current_pose = np.vstack((self.haptic_current_linear_pose, self.haptic_current_angular_pose))
        return self.initial_ee_pose + haptic_current_pose

    def Jointspace2GeometricJacobian(self):

        """ 
        Canculates geometrical jacobian.

        Parameters:
        self.joint_position_robot: 6x1 matrix containing joint angles of manipulator.

        Returns:
        J_geometrical: 6x6 matrix.
        """

        # End effector coordinate wrt to end effector frame     
        a_end = np.array([0, 0, 0.145, 1])

        # 1 = revolute joint, 0 = prismatic joint
        epsilon = np.array([1, 1, 1, 1, 1, 1])

        # DH parameters (classical), taken from UR official websit
        dh = np.array([[np.pi/2, 0, 0.15185, self.joint_position_robot[0,0]],
                       [0, -0.24355, 0, self.joint_position_robot[1,0]],
                       [0, -0.2132, 0, self.joint_position_robot[2,0]],
                       [np.pi/2, 0, 0.13105, self.joint_position_robot[3,0]],
                       [-np.pi/2, 0, 0.08535, self.joint_position_robot[4,0]],
                       [0, 0, 0.0921, self.joint_position_robot[5,0]]])

        # Loop to calculate coordinate of endeffector wrt base frame
        T = np.eye(4)
        for n in range(dh.shape[0]):
            alp, a, d, theta = dh[n]
            t = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alp), np.sin(theta) * np.sin(alp), a * np.cos(theta)],
                          [np.sin(theta), np.cos(theta) * np.cos(alp), -np.cos(theta) * np.sin(alp), a * np.sin(theta)],
                          [0, np.sin(alp), np.cos(alp), d],
                          [0, 0, 0, 1]])
            T = T @ t

        P = (T @ a_end)[:3]

        # Loop to calculate Jacobian
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

        # A snall constant to remove singularity
        lamda = 0.001

        # Concatenating Jv and Jw to get geometrical jacobian
        J_geometrical = np.vstack((Jv, Jw)) + lamda * np.eye(6)
        return J_geometrical
    
    def rpy2joint_space_vel(self):

        """ 
        Calculate joint space velocity.

        Parameters:
        roll, pitch, angles: roll, pitch, yaw angles of end effector.

        Returns:
        joint_space_velocity: 6x1 list.
        """

        alpha, beta, gamma = 0, 0, 0

        # A = np.array([[np.sin(beta), 0, 1],
        #               [-np.cos(beta) * np.sin(gamma), np.cos(gamma), 0],
        #               [np.cos(beta) * np.cos(gamma), np.sin(gamma), 0]])
        
        A = np.array([[1, 0            , np.sin(beta)               ], 
                      [0, np.cos(gamma), -np.cos(beta)*np.sin(gamma)], 
                      [0, np.sin(gamma), np.cos(beta)*np.cos(gamma) ]])

        J_geo2ana = np.block([[np.eye(3), np.zeros((3, 3))],
                              [np.zeros((3, 3)), np.linalg.inv(A)]])
        
        J_analytical = J_geo2ana @ self.Jointspace2GeometricJacobian()

        k = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])

        joint_space_velocity = np.linalg.inv(J_analytical) @ k @ (self.desired_traj() - self.current_ee_pose)

        return joint_space_velocity.flatten()
    
    def tf_callback(self, msg: TFMessage):

        """ 
        Callback function of /tf topic

        Parameters:
        TFMessage: end effector pose in task space [position, quaternion]

        Returns:
        self.current_ee_pose: 6x1 matrix containing pose of end effector at each time stamp.
        """

        # End effector pose in task space
        X_pos_ee = msg.transforms[0].transform.translation.x
        Y_pos_ee = msg.transforms[0].transform.translation.y
        Z_pos_ee = msg.transforms[0].transform.translation.z
        W_ori_ee = msg.transforms[0].transform.rotation.w
        X_ori_ee = msg.transforms[0].transform.rotation.x
        Y_ori_ee = msg.transforms[0].transform.rotation.y
        Z_ori_ee = msg.transforms[0].transform.rotation.z

        # Changing quaternion to euler 
        quat_from_robot = [W_ori_ee, X_ori_ee, Y_ori_ee, Z_ori_ee]
        euler_from_robot = self.quaternion_to_euler(quat_from_robot)

        # Current end effector pose in a 6x1 array
        current_ee_pose = np.array([[X_pos_ee], [Y_pos_ee], [Z_pos_ee], [euler_from_robot[0]], [euler_from_robot[1]], [euler_from_robot[2]]])

        if self.ee_start:
            self.initial_ee_pose = current_ee_pose
            self.ee_start = False
            #rospy.loginfo(f"End_effector_start: {self.end_effector_start}")

        self.current_ee_pose = current_ee_pose

    def haptic_callback(self, msg: PoseStamped):

        """ 
        Callback function of /phantom/phantom/pose topic

        Parameters:
        PoseStamped: stylus pose in task space [position, quaternion]

        Returns:
        self.haptic_current_linear_pose: 3x1 matrix containing linear pose of stylus.
        """

        # If condition to set initial offset of haptic device
        if self.haptic_start_1:
            self.h_offset_array = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                                            msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
            self.haptic_start_1 = False

        # Change in position = current position - initial position
        haptic_stylus_position = np.array([msg.pose.position.x - self.h_offset_array[0],
                                                msg.pose.position.y - self.h_offset_array[1],
                                                msg.pose.position.z - self.h_offset_array[2]])

        # Linear pose of stylus
        self.haptic_current_linear_pose = haptic_stylus_position.reshape(3, 1)

        # Publishing joint velocity to the manipulator
        desired_ee_trajectory = self.desired_traj()
        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = self.rpy2joint_space_vel()
        #self.velocity_pub.publish(velocity_pub_msg)

    def joint_callback_robot(self, msg: JointState):

        """ 
        Callback function of /joint_states topic

        Parameters:
        JointState: 1x6 list containing joint angles of manipulator.

        Returns:
        self.joint_position_context: 6x1 list containing joint angles of manipulator.
        """

        self.joint_position_robot = np.array(msg.position).reshape((6, 1))
        return self.joint_position_robot

    def haptic_offset(self, msg: JointState):

        """ 
        Callback function of /phantom/phantom/joint_states topic

        Parameters:
        JointState: 1x6 list containing joint angles of haptic device.

        Returns:
        self.haptic_current_angular_pose: 3x1 list containing joint angles of haptic device.
        """

        # Setting initial angular offset for haptic device
        if self.haptic_start_2:
            self.th_offsets = np.array([-msg.position[0], msg.position[1], -msg.position[2], msg.position[3], msg.position[4], msg.position[5]])
            self.haptic_start_2 = False

        # Forming array containg joint angles
        th_array = np.array([-msg.position[0] - self.th_offsets[0],
                             msg.position[1] - self.th_offsets[1],
                             msg.position[2] + self.th_offsets[2],
                             (-msg.position[3] + self.th_offsets[3]) if msg.position[3] > 0 else (msg.position[3] + self.th_offsets[3]),
                             msg.position[4] - self.th_offsets[4] if msg.position[4] != 0 else 0.0,
                             msg.position[5] - self.th_offsets[5] if msg.position[5] != 0 else 0.0])

        # Calculating rotation matrix
        q = self.Q(th_array)
        
        # Calculating quaternion from rotation matrix
        q0_h = math.sqrt(1 + q[0, 0] + q[1, 1] + q[2, 2]) / 2
        if q0_h != 0:
            q1_h = (q[2, 1] - q[1, 2]) / (4 * q0_h)
            q2_h = (q[0, 2] - q[2, 0]) / (4 * q0_h)
            q3_h = (q[1, 0] - q[0, 1]) / (4 * q0_h)
        else:
            q1_h = math.sqrt((q[0, 0] + 1)/2)
            q2_h = math.sqrt((q[1, 1] + 1)/2)
            q3_h = math.sqrt((q[2, 2] + 1)/2)

        # Calculating euler angles from quaternion
        haptic_quat = [q0_h, q1_h, q2_h, q3_h]
        self.haptic_current_angular_pose = np.array(self.quaternion_to_euler(haptic_quat)).reshape(3, 1)


    def main_loop(self): 
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = RobotEndEffectorController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass