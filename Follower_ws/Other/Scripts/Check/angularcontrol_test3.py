#!/usr/bin/env python3

import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import math

global ee_start2 
ee_start2 = True

def quaternion_to_euler(quat):
    """Convert quaternion to Euler angles."""
    rotation = R.from_quat([quat[1], quat[2], quat[3], quat[0]])  # Correct order: [x, y, z, w]
    return rotation.as_euler('xyz', degrees=False)

def dead_band_th(theta_array):
    """Apply deadband to theta array."""
    return [0.0 if abs(j) < 0.0014 else j for j in theta_array]

def Qx(th):
    qx = np.array([[1, 0, 0],[0, math.cos(th), -math.sin(th)],[0, math.sin(th), math.cos(th)]])
    return qx

def Qy(th):
    qy = np.array([[math.cos(th), 0, math.sin(th)],[0, 1, 0],[-math.sin(th), 0, math.cos(th)]])
    return qy

def Qz(th):
    qz = np.array([[math.cos(th), -math.sin(th), 0],[math.sin(th), math.cos(th), 0], [0, 0, 1]])
    return qz

def Q(theta_q):
    q1 = Qz(theta_q[0])
    q2 = Qx(theta_q[1])
    q3 = Qx(theta_q[2])
    q4 = Qz(theta_q[3])
    q5 = Qx(theta_q[4])
    q6 = Qy(theta_q[5])
    q = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(q1,q2),q3),q4),q5),q6)
    return q

def tf_callback(msg: TFMessage, context):
    """Robot end-effector POSE callback."""
    ee_start, robot_offset, velocity_pub, current_ee_pose = context

    global ee_start2

    X_pos_ee = msg.transforms[0].transform.translation.x
    Y_pos_ee = msg.transforms[0].transform.translation.y
    Z_pos_ee = msg.transforms[0].transform.translation.z

    W_ori_ee = msg.transforms[0].transform.rotation.w
    X_ori_ee = msg.transforms[0].transform.rotation.x
    Y_ori_ee = msg.transforms[0].transform.rotation.y
    Z_ori_ee = msg.transforms[0].transform.rotation.z

    quat_from_robot = [W_ori_ee, X_ori_ee, Y_ori_ee, Z_ori_ee]
    euler_from_robot = quaternion_to_euler(quat_from_robot)

    if X_pos_ee != robot_offset[0]:
        current_ee_pose = np.array([[X_pos_ee], [Y_pos_ee], [Z_pos_ee],
                         [euler_from_robot[0]], [euler_from_robot[1]], [euler_from_robot[2]]])

        if ee_start2:
            initial_ee_pose = current_ee_pose
            ee_start2 = False
            rospy.loginfo(f"initial_ee_pose: {initial_ee_pose}")
    print("execute ho ragha hai")
    context[3][:] = current_ee_pose  # Update the shared current_ee_pose in context
    print("current ee pose : ", context[3][:])

def haptic_callback(msg: PoseStamped, context):
    """Haptic stylus position callback.\n
    We are using linear positions and not utilizing orientation values as it is incorrect"""
    haptic_stylus_position, h_offset_array, flag_pos_offset, haptic_current_linear_pose = context

    if flag_pos_offset[0]:
        h_offset_array[:] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                             msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]
        flag_pos_offset[0] = False

    haptic_stylus_position[0] = msg.pose.position.x - h_offset_array[0]
    haptic_stylus_position[1] = msg.pose.position.y - h_offset_array[1]
    haptic_stylus_position[2] = msg.pose.position.z - h_offset_array[2]

    haptic_current_linear_pose[:] = np.array([[haptic_stylus_position[0]], [haptic_stylus_position[1]], [haptic_stylus_position[2]]])

def joint_callback_robot(msg: JointState, context):
    """Joint state callback."""
    joint_position_context = context
    joint_position_context[:] = msg.position

def haptic_offset(msg, context):
    """Haptic offset callback."""
    th1_h_off, th2_h_off, th3_h_off, th4_h_off, th5_h_off, th6_h_off, flag_offset, haptic_current_angular_pose = context
    q0_h, q1_h, q2_h, q3_h, th_array, haptic_angular = [0], [0], [0], [0], [], [0]

    if flag_offset[0]:
        th1_h_off[0] = -msg.position[0]
        th2_h_off[0] = msg.position[1]
        th3_h_off[0] = -msg.position[2]
        th4_h_off[0] = msg.position[3]
        th5_h_off[0] = msg.position[4]
        th6_h_off[0] = msg.position[5]

        print(f"Inside flag : {flag_offset}")
        flag_offset[0] = False

    th1_h = -msg.position[0] - th1_h_off[0]
    th2_h = msg.position[1] - th2_h_off[0]
    th3_h = msg.position[2] + th3_h_off[0]

    th4_h = -msg.position[3] + th4_h_off[0] if msg.position[3] > 0 else msg.position[3] + th4_h_off[0]
    th5_h = msg.position[4] - th5_h_off[0] if msg.position[4] != 0 else 0.0
    th6_h = msg.position[5] - th6_h_off[0] if msg.position[5] != 0 else 0.0

    th_array[:] = [th1_h, th2_h, th3_h, th4_h, th5_h, th6_h]
    THETA_SEND = dead_band_th(th_array)

    q = Q(THETA_SEND)  # Q is a function to generate rotation matrix 3x3
    
    q0_h[0] = math.sqrt(1 + q[0, 0] + q[1, 1] + q[2, 2]) / 2
    if q0_h != 0:
        q1_h[0] = (q[2, 1] - q[1, 2]) / (4 * q0_h[0])
        q2_h[0] = (q[0, 2] - q[2, 0]) / (4 * q0_h[0])
        q3_h[0] = (q[1, 0] - q[0, 1]) / (4 * q0_h[0])
    else:
        q1_h[0] = math.sqrt((q[0, 0] + 1)/2)
        q2_h[0] = math.sqrt((q[1, 1] + 1)/2)
        q3_h[0] = math.sqrt((q[2, 2] + 1)/2)

    print("th array: ", th_array)

    haptic_quat = [q0_h[0], q1_h[0], q2_h[0], q3_h[0]]
    haptic_euler_from_quat = quaternion_to_euler(haptic_quat)
    haptic_angular[:] = haptic_euler_from_quat
    print("Haptic angular: ", haptic_angular)

    haptic_current_angular_pose[:] = np.array([[haptic_angular[0]], [haptic_angular[1]], [haptic_angular[2]]])

def desired_traj(haptic_current_linear_pose, haptic_current_angular_pose, initial_ee_pose):
    haptic_current_pose = np.vstack((haptic_current_linear_pose, haptic_current_angular_pose))
    desired_ee_trajectory = initial_ee_pose + haptic_current_pose
    print("desired traj: ", desired_ee_trajectory)
    print("haptic current pose: ", haptic_current_pose)
    return desired_ee_trajectory

def Jointspace2GeometricJacobian(joint_position_robot):
    # Coordinate of end effector in nth link frame
    a_end = np.array([0, 0, 0.145, 1])

    # Joint type: 1 for revolute, 0 for prismatic
    epsilon = np.array([1, 1, 1, 1, 1, 1])

    # Denavit-Hartenberg parameters
    dh = np.array([[np.pi/2 , 0       , 0.15185, joint_position_robot[0]],  
                   [0       , -0.24355, 0      , joint_position_robot[1]],
                   [0       , -0.2132 , 0      , joint_position_robot[2]],
                   [np.pi/2 , 0       , 0.13105, joint_position_robot[3]],
                   [-np.pi/2, 0       , 0.08535, joint_position_robot[4]],
                   [0       , 0       , 0.0921 , joint_position_robot[5]]])
    
    # Number of links
    n_links = dh.shape[0]

    # Finding homogeneous transformation matrix
    T = np.eye(4)

    for n in range(n_links):

        alp = dh[n,0]
        a = dh[n,1]
        d = dh[n,2]
        theta = dh[n,3]

        t = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alp), np.sin(theta) * np.sin(alp) , a * np.cos(theta)],
                      [np.sin(theta), np.cos(theta) * np.cos(alp) , -np.cos(theta) * np.sin(alp), a * np.sin(theta)],
                      [0            , np.sin(alp)                 , np.cos(alp)                 , d                ],
                      [0            , 0                           , 0                           , 1                ]])
        
        T = np.dot(T, t)

    p = np.dot(T, a_end)
    P = p[:3] # Position matrix of end effector with respect to base

    # Finding Jacobian matrix
    T1 = np.eye(4)
    Jv = np.zeros((3, n_links))
    Jw = np.zeros((3, n_links))

    for n in range(n_links):

        alp = dh[n,0]
        a = dh[n,1]
        d = dh[n,2]
        theta = dh[n,3]

        t = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alp), np.sin(theta) * np.sin(alp) , a * np.cos(theta)],
                      [np.sin(theta), np.cos(theta) * np.cos(alp) , -np.cos(theta) * np.sin(alp), a * np.sin(theta)],
                      [0            , np.sin(alp)                 , np.cos(alp)                 , d                ],
                      [0            , 0                           , 0                           , 1                ]])
        
        T1 = np.dot(T1, t)

        Z = T1[:3, 2]
        oo = T1[:3, 3]
        Jv[:, n] = epsilon[n] * np.cross(Z, P - oo) 
        Jw[:, n] = epsilon[n] * Z
    
    lamda = 0.001 # A small constant to avoid singularity
    J_geometrical = np.vstack((Jv, Jw)) + lamda*np.eye(6) # Geometric Jacobian matrix

    return J_geometrical

def rpy2joint_space_vel(alpha, beta, gamma, current_ee_pose, desired_ee_trajectory, joint_position_robot):
    # The matrix A is itself a Jacobian that maps XYZ roll-pitch-yaw angle velocity to angular velocity
    # Euler angle set XYZ
    # roll - z, pitch - y, yaw - x
    A = np.array([[np.sin(beta)               , 0            , 1], 
                  [-np.cos(beta)*np.sin(gamma), np.cos(gamma), 0], 
                  [np.cos(beta)*np.cos(gamma) , np.sin(gamma), 0]])

    # Matrix which will convert geometric jacobian to analytical jacobian
    J_geo2ana = np.block([[np.eye(3)       , np.zeros((3, 3))],
                        [np.zeros((3, 3)), np.linalg.inv(A)]])

    # Calculating analytical jacobian 
    J_analytical = J_geo2ana @ Jointspace2GeometricJacobian(joint_position_robot)

    k = np.array([[1, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0]])
    joint_space_velocity = np.linalg.inv(J_analytical) @ k @ (desired_ee_trajectory - current_ee_pose)
    return joint_space_velocity.flatten()  # Ensure it's a 1D array for publishing


def main():
    
    rospy.init_node('robot_end_effector_controller', anonymous=True)
    
    velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)

    ee_start = [True]
    robot_offset = np.array([-0.24355, 0, 0, 0, 0, 0, 0])   
    current_ee_pose = np.zeros((6, 1))  # Initialize current_ee_pose as shared state
    initial_ee_pose = np.zeros((6,1))
    tf_context = (ee_start, robot_offset, velocity_pub, current_ee_pose)

    haptic_stylus_position = np.zeros(3)
    h_offset_array = np.zeros(7)
    flag_pos_offset = [True]
    haptic_current_linear_pose = np.zeros((3, 1))  # Initialize haptic_current_linear_pose as shared state
    haptic_context = (haptic_stylus_position, h_offset_array, flag_pos_offset, haptic_current_linear_pose)

    joint_position_context = np.zeros(6)  # Initialize joint_position_context as shared state

    th1_h_off, th2_h_off, th3_h_off, th4_h_off, th5_h_off, th6_h_off = [np.zeros(1) for _ in range(6)]
    flag_offset = [True]
    haptic_current_angular_pose = np.zeros((3, 1))  # Initialize haptic_current_angular_pose as shared state
    haptic_offset_context = (th1_h_off, th2_h_off, th3_h_off, th4_h_off, th5_h_off, th6_h_off, flag_offset, haptic_current_angular_pose)

    rospy.Subscriber('/tf', TFMessage, tf_callback, callback_args=tf_context)
    rospy.Subscriber('/phantom/phantom/pose', PoseStamped, haptic_callback, callback_args=haptic_context)
    rospy.Subscriber('/joint_states', JointState, joint_callback_robot, callback_args=joint_position_context)
    rospy.Subscriber('/phantom/phantom/joint_states', JointState, haptic_offset, callback_args=haptic_offset_context)

    rate = rospy.Rate(500)  # 500 Hz update rate
    while not rospy.is_shutdown():
        desired_ee_trajectory = desired_traj(haptic_current_linear_pose, haptic_current_angular_pose, initial_ee_pose)

        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = rpy2joint_space_vel(0, 0, 0, current_ee_pose, desired_ee_trajectory, joint_position_context)  # alpha, beta, gamma should be defined or computed
        print("joint velocity published: ",velocity_pub_msg.data)
        #velocity_pub.publish(velocity_pub_msg)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass