#!/usr/bin/env python3

import rospy
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray,  queue_size = 10)











def dead_band_th(theta_array):
    for j in theta_array: 

        if abs(j) < 0.0014:
            j = 0.0

    return theta_array





def Qx(th):
    qx = np.matrix([[1, 0, 0],[0, math.cos(th), -math.sin(th)],[0, math.sin(th), math.cos(th)]])
    return qx

def Qy(th):
    qy = np.matrix([[math.cos(th), 0, math.sin(th)],[0, 1, 0],[-math.sin(th), 0, math.cos(th)]])
    return qy

def Qz(th):
    qz = np.matrix([[math.cos(th), -math.sin(th), 0],[math.sin(th), math.cos(th), 0], [0, 0, 1]])
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



def quaternion_to_euler(q):
    """
    Convert a quaternion into XYZ Euler angles (in radians).
    
    Parameters:
    q (list or numpy array): A quaternion [qx, qy, qz, qw]
    
    Returns:
    euler_angles (numpy array): The XYZ Euler angles [roll, pitch, yaw] in radians
    """
    # Create a Rotation object from the quaternion
    r = R.from_quat(q)
    
    # Convert the Rotation object to Euler angles
    euler_angles = r.as_euler('xyz', degrees=False)
    
    return euler_angles

########################################               Example quaternion to Euler angles

# # Example quaternion [qx, qy, qz, qw]
# quaternion = [0.1, 0.2, 0.3, 0.9]

# # Convert quaternion to XYZ Euler angles
# euler_angles = quaternion_to_euler(quaternion)

# # Print the result
# print("XYZ Euler angles (radians):", euler_angles)
# print("XYZ Euler angles (degrees):", np.degrees(euler_angles))
###############################################################################################################################

def tf_callback(msg):    

    """ robot end-effector POSE """
    
    global ee_start, end_effector_start,robot_ee_pose

    robot_offset = [-0.24355,0,0,0,0,0,0]

    
    X_pos_ee = msg.transforms[0].transform.translation.x
    Y_pos_ee = msg.transforms[0].transform.translation.y
    Z_pos_ee = msg.transforms[0].transform.translation.z

    W_ori_ee = msg.transforms[0].transform.rotation.w
    X_ori_ee = msg.transforms[0].transform.rotation.x
    Y_ori_ee = msg.transforms[0].transform.rotation.y
    Z_ori_ee = msg.transforms[0].transform.rotation.z

    
    quat_from_robot = [W_ori_ee,X_ori_ee,Y_ori_ee,Z_ori_ee]
    euler_from_robot = quaternion_to_euler(quat_from_robot)  ######## Check whether X is roll or yaw.

    
    if X_pos_ee != -0.24355:
        robot_ee_pose = np.array([X_pos_ee,Y_pos_ee,Z_pos_ee,euler_from_robot[0],euler_from_robot[1],euler_from_robot[2]])
        robot_ee_pose_transpose = np.transpose(robot_ee_pose)

    # if robot_ee_pose[0] - robot_offset[0] != 0.0:
    if ee_start == True:
        end_effector_start = robot_ee_pose
        ee_start = False

        print("End_effector_start; ",end_effector_start)




def haptic_callback(msg):
    global haptic_stylus_position, h_offset_array, flag_pos_offset, haptic_angular , haptic_stylus_orientation ,  roll_haptic,pitch_haptic,yaw_haptic 

   
    if flag_pos_offset == True:

        h_offset_array[0] = msg.pose.position.x
        h_offset_array[1] = msg.pose.position.y 
        h_offset_array[2] = msg.pose.position.z 


        h_offset_array[3] = msg.pose.orientation.w
        h_offset_array[4] = msg.pose.orientation.x
        h_offset_array[5] = msg.pose.orientation.y
        h_offset_array[6] = msg.pose.orientation.z

        
        flag_pos_offset = False

    haptic_stylus_position[0] = msg.pose.position.x - h_offset_array[0]
    haptic_stylus_position[1] = msg.pose.position.y - h_offset_array[1] 
    haptic_stylus_position[2] = msg.pose.position.z - h_offset_array[2] 

def haptic_offset(msg3):
    global th1_h, th2_h, th3_h, th4_h, th5_h, th6_h, th1_h_off,th2_h_off, th3_h_off, th4_h_off,th5_h_off,th6_h_off
    global q0_h,q1_h,q2_h,q3_h,flag_offset,th_array, haptic_angular ,haptic_stylus_orientation 
 
    if flag_offset == True:

        th1_h_off  = -msg3.position[0]
        th2_h_off  = msg3.position[1]
        th3_h_off  = -msg3.position[2]
        th4_h_off  = msg3.position[3]
        th5_h_off  = msg3.position[4]
        th6_h_off  = msg3.position[5]
    

        print(f"Inside flag : {flag_offset}")
        flag_offset = False
        
    
    th1_h = -msg3.position[0] - th1_h_off
    th2_h = msg3.position[1] - th2_h_off
    th3_h = msg3.position[2] + th3_h_off
    
    if msg3.position[3] > 0:
        th4_h = -msg3.position[3] + th4_h_off ############ !!!
    elif msg3.position[3] < 0:
        th4_h = msg3.position[3] + th4_h_off
    else:
        th4_h = 0.0



    if msg3.position[4] > 0:
        th5_h = msg3.position[4] - th5_h_off
    if msg3.position[4] < 0:
        th5_h = msg3.position[4] - th5_h_off
    else:
        th5_h = 0.0



    if msg3.position[5] > 0:
        th6_h = msg3.position[5] - th6_h_off ############ !!!   
    if msg3.position[5] < 0:    
        th6_h = msg3.position[5] - th6_h_off
    else:
        th6_h = 0.0

   
    th_array = [th1_h,th2_h,th3_h,th4_h,th5_h,th6_h]
    THETA_SEND = dead_band_th(theta_array=th_array)
    
    q = Q(THETA_SEND)    ## q is a rotation matrix of 3x3 generated by Q function
   
    q0_h = math.sqrt(1 + q[0,0] + q[1,1] + q[2,2]) /2  ## converting the rotation matrix to quaterniun [w, x, y, z]
    q1_h = (q[2,1] - q[1,2])/( 4 *q0_h)
    q2_h = (q[0,2] - q[2,0])/( 4 *q0_h)
    q3_h = (q[1,0] - q[0,1])/( 4 *q0_h)
    
    
    print("th array: ", th_array)


    # Compute roll, pitch, and yaw from the rotation matrix q
    sy = np.sqrt(q[2, 2] ** 2 + q[2, 1] ** 2)
    singular = sy < 1e-6

    # if not singular:
    #     roll_from_matrix = np.arctan2(q[2, 1], q[2, 2])
    #     pitch_from_matrix = np.arctan2(-q[2, 0], sy)
    #     yaw_from_matrix = np.arctan2(q[1, 0], q[0, 0])
    # else:
    #     roll_from_matrix = np.arctan2(-q[1, 2], q[1, 1])
    #     pitch_from_matrix = np.arctan2(-q[2, 0], sy)
    #     yaw_from_matrix = 0
    
    
    haptic_quat = [q0_h,q1_h,q2_h,q3_h]
    haptic_euler_from_quat = quaternion_to_euler(haptic_quat)
    haptic_angular = np.array(haptic_euler_from_quat)
    print("Haptic angular: ",haptic_angular)




def joint_callback_robot(msg):
    
    
    global joint_position
    joint_position = msg.position


def desired_traj(xh,yh,zh,roll_h,pitch_h,yaw_h):
    
    """ Set your goal position at robot end and not on haptic device end.
    \n roll_h,pitch_h,yaw_h refers to angular position of haptic end effector. """


    global flag2,h_store_x,h_store_y,h_store_z
    global des_traj_array

    global end_effector_start, robot_quaternion
    global h_angular_store_x, h_angular_store_y, h_angular_store_z
    

    #if zh < 100 or phi_h < 100 or theta_h < 100 or psi_h < 100:

    x_0 = xh + end_effector_start[0] 
    y_0 = yh + end_effector_start[1] 
    z_0 = zh + end_effector_start[2] 
           

    roll_desired = roll_h + end_effector_start[3]
    pitch_desired = pitch_h + end_effector_start[4] 
    yaw_desired = yaw_h + end_effector_start[5]
    


    print(f"z_0: {z_0}")
    print(f"z_h: {zh}")
    print(f"y_h: {yh}")
    print(f"x_h: {xh}")
    print(f"yaw_desired: {yaw_desired}")

    
    
    des_traj_array = np.array([x_0,y_0,z_0,yaw_desired,pitch_desired,roll_desired])
    print("desired traj: ", des_traj_array)
    return des_traj_array


# Function to calculate geometrical jacobian twith the help of joint angles
# Input - joint angles in form of a (6x1) matrix
# Output - geometrical jacobian in form of a (6x6) matrix
def Jointspace2GeometricJacobian(q):

    global joint_position

    # Coordinate of end effector in nth link frame
    a = np.array([0, 0, 0.145, 1])

    # Joint type: 1 for revolute, 0 for prismatic
    epsilon = np.array([1, 1, 1, 1, 1, 1])

    # Denavit-Hartenberg parameters
    # dh = np.array([[np.pi/2 , 0       , 0.15185, q[0,0]],  
    #                [0       , -0.24355, 0      , q[1,0]],
    #                [0       , -0.2132 , 0      , q[2,0]],
    #                [np.pi/2 , 0       , 0.13105, q[3,0]],
    #                [-np.pi/2, 0       , 0.08535, q[4,0]],
    #                [0       , 0       , 0.0921 , q[5.0]]])
    
    dh = np.array([[np.pi/2 , 0       , 0.15185, joint_position[0]],  
                   [0       , -0.24355, 0      , joint_position[1]],
                   [0       , -0.2132 , 0      , joint_position[2]],
                   [np.pi/2 , 0       , 0.13105, joint_position[3]],
                   [-np.pi/2, 0       , 0.08535, joint_position[4]],
                   [0       , 0       , 0.0921 , joint_position[5]]])
    
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

    p = np.dot(T, a)
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
        
        T1 = np.dot(T, t)

        Z = T1[:3, 2]
        oo = T1[:3, 3]
        Jv[:, n] = epsilon[n] * np.cross(Z, P - oo) + (1 - epsilon[n]) * Z
        Jw[:, n] = epsilon[n] * Z
    
    lamda = 0.001 # A small constant to avoid singularity
    J_geometrical = np.vstack((Jv, Jw)) + lamda*np.eye(6) # Geometric Jacobian matrix

    return J_geometrical

# alpha - angle about z axis
# beta - angle about y axis
# gamma - angle about x axis
global alpha, beta, gamma


def rpy2joint_space_vel(alpha, beta, gamma, current_ee_pose, next_ee_pose):
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
    J_analytical = J_geo2ana * Jointspace2GeometricJacobian(q)

    k = np.eye(6)
    joint_space_velocity = np.linalg.inv(J_analytical)*k*(next_ee_pose - current_ee_pose)
    return np.transpose(joint_space_velocity)


def main():
    rospy.Subscriber('/tf', TFMessage, tf_callback)    ###    Robot end effector pose
    rospy.Subscriber('/phantom/phantom/pose', PoseStamped, haptic_callback)          # haptic end effector position only, orientation wrong         
    rospy.Subscriber('/phantom/phantom/joint_states', JointState, haptic_offset)      # haptic joint position 
    rospy.Subscriber('/joint_states', JointState, joint_callback_robot)

    # Publishing joint velocity
    velocity_pub_msg = Float64MultiArray()
    velocity_pub_msg.data = rpy2joint_space_vel(alpha, beta, gamma, task_space_pose)
    velocity_pub.publish(velocity_pub_msg)


if __name__ == "__main__":

    try:
        main()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass