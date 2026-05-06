#!/usr/bin/env python3

import rospy
import numpy as np
import sys
import actionlib
from tf2_msgs.msg import TFMessage
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal, CartesianTrajectoryPoint,FollowCartesianTrajectoryActionFeedback
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped,TwistStamped
import geometry_msgs.msg as geometry_msgs
import time
import math
import os

global teleop_file

global h_offset_array, haptic_angular,haptic_stylus_pose, position_sent_to_robot,quat_position_sent_to_robot

global ee_start,flag_pos_offset,flag_offset, flag2

global des_traj_array

teleop_file = 'Teleop Position'

ee_start = True
flag_pos_offset = True
flag_offset = True
flag2 = True

haptic_stylus_pose = np.empty(6)
h_offset_array = np.empty(3)
haptic_angular = np.empty(4)
robot_ee_pose = np.empty(7)
end_effector_start = np.empty(7)
des_traj_array = np.empty(7)

position_sent_to_robot = []
quat_position_sent_to_robot = []



cartesian_trajectory_controller = "pose_based_cartesian_traj_controller"



def euler_to_quat(roll_arg,pitch_arg,yaw_arg):
    r = R.from_euler('xyz',[roll_arg,pitch_arg,yaw_arg],degrees=False)
    quat = r.as_quat()
    return quat

def quat2euler(scalar_angle, vec_1, vec_2, vec_3):
    phi = math.atan2(2*(scalar_angle*vec_1 + vec_2*vec_3),1-2*((vec_1)**2+(vec_2)**2))
    th  = math.asin(2*(scalar_angle*vec_2 - vec_3*vec_1))
    psi = math.atan2(2*(scalar_angle*vec_3 + vec_1*vec_2),1-2*((vec_2)**2+(vec_3)**2))
    return phi, th, psi


def joint_position_callback(msg):
    
    global joint_position
    joint_position =  msg.position 


def forward_kinematics():
    global joint_position
    
    
    n = 6

    alp = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
    a = [0, -0.24355, -0.2132, 0, 0, 0]
    d = [0.15185, 0, 0, 0.13105, 0.08535, 0.0921]
    
    th = ([joint_position[2], joint_position[1],joint_position[0],joint_position[3], joint_position[4], joint_position[5]])


    # Value of d_E_n
    d_E_n = np.array([0, 0, 0.145])

    # Initialization
    T_i = np.eye(4)

    R = np.zeros((3, 3, n))
    O = np.zeros((3, n))

    # Forward kinematics
    for i in range(n):
        T_i_i_m = np.array([
            [np.cos(th[i]), -np.sin(th[i]) * np.cos(alp[i]), np.sin(th[i]) * np.sin(alp[i]), a[i] * np.cos(th[i])],
            [np.sin(th[i]), np.cos(th[i]) * np.cos(alp[i]), -np.cos(th[i]) * np.sin(alp[i]), a[i] * np.sin(th[i])],
            [0, np.sin(alp[i]), np.cos(alp[i]), d[i]],
            [0, 0, 0, 1]
        ])
        
        T_i_0 = T_i @ T_i_i_m
        R_i_0 = T_i_0[:3, :3]
        o_i_0 = T_i_0[:3, 3]
        R[:, :, i] = R_i_0
        O[:, i] = o_i_0
        T_i = T_i_0

    T_n_0 = T_i_0
    P_E_0 = T_n_0 @ np.append(d_E_n, 1)
    p_E_0 = P_E_0[:3]

    # Extract rotation matrix from T_n_0
    R_n_0 = T_n_0[:3, :3]

    # Compute roll, pitch, and yaw from the rotation matrix
    sy = np.sqrt(R_n_0[2, 2] ** 2 + R_n_0[2, 1] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(R_n_0[2, 1], R_n_0[2, 2])
        pitch = np.arctan2(-R_n_0[2, 0], sy)
        yaw = np.arctan2(R_n_0[1, 0], R_n_0[0, 0])
    else:
        roll = np.arctan2(-R_n_0[1, 2], R_n_0[1, 1])
        pitch = np.arctan2(-R_n_0[2, 0], sy)
        yaw = 0


    # Display results
    print('T_6_0 is:')
    print(T_n_0)

    print('Position at end effector at (0,0,0.145):')
    print(p_E_0)

    print('Roll, Pitch, Yaw:')
    print('Roll:', roll)
    print('Pitch:', pitch)
    print('Yaw:', yaw)   


def Qx(th):
    qx = np.matrix([[1, 0, 0],[0, math.cos(th), -math.sin(th)],[0, math.sin(th), math.cos(th)]])
    return qx

def Qy(th):
    qy = np.matrix([[math.cos(th), 0, math.sin(th)],[0, 1, 0],[-math.sin(th), 0, math.cos(th)]])
    return qy

def Qz(th):
    qz = np.matrix([[math.cos(th), -math.sin(th), 0],[math.sin(th), math.cos(th), 0], [0, 0, 1]])
    return qz

def Q(th1, th2, th3, th4, th5, th6):
    

    q1 = Qz(th1)
    q2 = Qx(th2)
    q3 = Qx(th3)
    q4 = Qz(th4)
    
    q5 = Qx(th5)

    q6 = Qy(th6)

    q = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(q1,q2),q3),q4),q5),q6)
    return q

def haptic_pose_callback(msg):
    
    global h_offset_array,haptic_angular,haptic_stylus_pose,flag_pos_offset


    if flag_pos_offset == True:

        h_offset_array[0] = msg.pose.position.x
        h_offset_array[1] = msg.pose.position.y 
        h_offset_array[2] = msg.pose.position.z 
        
        flag_pos_offset = False
            
    haptic_stylus_pose[0] = msg.pose.position.x - h_offset_array[0]
    haptic_stylus_pose[1] = msg.pose.position.y - h_offset_array[1] 
    haptic_stylus_pose[2] = msg.pose.position.z - h_offset_array[2] 

    haptic_stylus_pose = np.array([haptic_stylus_pose[0],haptic_stylus_pose[1],haptic_stylus_pose[2],haptic_angular[0],haptic_angular[1],haptic_angular[2],haptic_angular[3]])
    print("Haptic stylus position: ",haptic_stylus_pose)


def haptic_offset(msg3):
    global th1_h, th2_h, th3_h, th4_h, th5_h, th6_h,th1_h_off,th2_h_off, th3_h_off, th4_h_off,th5_h_off,th6_h_off
    global q0_h,q1_h,q2_h,q3_h,flag_offset,th_array, haptic_angular
    

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
    
    q = Q(th1_h, th2_h, th3_h, th4_h, th5_h, th6_h)    ## q is a rotation matrix of 3x3 generated by Q function
   
    q0_h = math.sqrt(1 + q[0,0] + q[1,1] + q[2,2]) /2  ## converting the rotation matrix to quaterniun [w, x, y, z]
    q1_h = (q[2,1] - q[1,2])/( 4 *q0_h)
    q2_h = (q[0,2] - q[2,0])/( 4 *q0_h)
    q3_h = (q[1,0] - q[0,1])/( 4 *q0_h)
    


    th_array = [th1_h,th2_h,th3_h,th4_h,th5_h,th6_h]

    haptic_angular = np.array([q0_h,q1_h,q2_h,q3_h])


# def end_eff_cb(msg):

#     global ee_start, end_effector_start,robot_ee_pose

#     X_pos_ee = msg.feedback.actual.pose.position.x
#     Y_pos_ee = msg.feedback.actual.pose.position.y
#     Z_pos_ee = msg.feedback.actual.pose.position.z



#     W_ori_ee = msg.feedback.actual.pose.orientation.w
#     X_ori_ee = msg.feedback.actual.pose.orientation.x
#     Y_ori_ee = msg.feedback.actual.pose.orientation.y
#     Z_ori_ee = msg.feedback.actual.pose.orientation.z

#     robot_ee_pose = np.array([X_pos_ee,Y_pos_ee,Z_pos_ee,W_ori_ee,X_ori_ee,Y_ori_ee,Z_ori_ee])

#     if ee_start == True:
#         end_effector_start = robot_ee_pose
#         ee_start = False

#     print("End_effector_start; ",end_effector_start)
    
def tf_callback(msg):
    
    global ee_start, end_effector_start,robot_ee_pose
    
    X_pos_ee = msg.transforms[0].transform.translation.x
    Y_pos_ee = msg.transforms[0].transform.translation.y
    Z_pos_ee = msg.transforms[0].transform.translation.z

    W_ori_ee = msg.transforms[0].transform.rotation.w
    X_ori_ee = msg.transforms[0].transform.rotation.x
    Y_ori_ee = msg.transforms[0].transform.rotation.y
    Z_ori_ee = msg.transforms[0].transform.rotation.z

    
    robot_ee_pose = np.array([X_pos_ee,Y_pos_ee,Z_pos_ee,W_ori_ee,X_ori_ee,Y_ori_ee,Z_ori_ee])

    if ee_start == True:
        end_effector_start = robot_ee_pose
        ee_start = False

    print("End_effector_start; ",end_effector_start)



def desired_traj(xh,yh,zh,w_hap,x_hap,y_hap,z_hap):
    
    """ Set your goal position at robot end and not on haptic device end """


    global flag2,h_store_x,h_store_y,h_store_z
    global des_traj_array

    global end_effector_start, robot_quaternion
    global h_angular_store_x, h_angular_store_y, h_angular_store_z
    

    #if zh < 100 or phi_h < 100 or theta_h < 100 or psi_h < 100:
    if flag2 == True:
        x_0 = xh + end_effector_start[0] 
        y_0 = yh + end_effector_start[1] 
        z_0 = zh + end_effector_start[2]                 


        # x_ang = phi_h +  end_effector_start[3] 
        # y_ang = theta_h + end_effector_start[4]
        # z_ang = psi_h + end_effector_start[5]

        w_quat = w_hap + end_effector_start[3]
        x_quat = x_hap + end_effector_start[4] 
        y_quat = y_hap + end_effector_start[5]
        z_quat = z_hap + end_effector_start[6]


        print(f"z_0: {z_0}")
        print(f"z_quat: {z_quat}")

    
    
    des_traj_array = np.array([x_0,y_0,z_0,w_quat,x_quat,y_quat,z_quat])
    print("desired traj: ", des_traj_array)
    return des_traj_array


def send_cartesian_trajectory():
    

   
    rospy.Subscriber('/joint_states', JointState, joint_position_callback)
    
    rospy.Subscriber('/phantom/pose', PoseStamped, haptic_pose_callback)
    
    rospy.Subscriber('/phantom/joint_states',JointState, haptic_offset)
    rospy.Subscriber('/tf',TFMessage,tf_callback)
    #rospy.Subscriber('/pose_based_cartesian_traj_controller/follow_cartesian_trajectory/feedback',FollowCartesianTrajectoryActionFeedback,end_eff_cb)
    
    
    global des_traj_array, cartesian_trajectory_controller, haptic_stylus_pose
    
    linear_pos = np.array([des_traj_array[0],des_traj_array[1],des_traj_array[2]])
    quat_pos = np.array([des_traj_array[3],des_traj_array[4],des_traj_array[5],des_traj_array[6]])

    
#while not rospy.is_shutdown():
    pose_list_obtained = desired_traj(haptic_stylus_pose[0],haptic_stylus_pose[1],haptic_stylus_pose[2],haptic_stylus_pose[3],haptic_stylus_pose[4],haptic_stylus_pose[5],haptic_stylus_pose[6])

    pose_list = geometry_msgs.Pose(geometry_msgs.Vector3(pose_list_obtained[0], pose_list_obtained[1], pose_list_obtained[2]), geometry_msgs.Quaternion(pose_list_obtained[3], pose_list_obtained[4], pose_list_obtained[5], pose_list_obtained[6]))

    
    #rospy.sleep(2)  # Wait for action server to start
    goal = FollowCartesianTrajectoryGoal()
    trajectory_client = actionlib.SimpleActionClient(
        "{}/follow_cartesian_trajectory".format(cartesian_trajectory_controller),
        FollowCartesianTrajectoryAction,
    )
    # timeout = rospy.Duration(1)
    # if not trajectory_client.wait_for_server(timeout):
    #     rospy.logerr("Could not reach controller action server.")
    #     sys.exit(-1)
    # if linear_pos is None or linear_pos.size != 3:
    #     rospy.logerr("Linear position is not set correctly.")
    #     sys.exit(-1)
    point = CartesianTrajectoryPoint()
    
    # point.pose.position = Point(*linear_pos)
    # point.pose.orientation = Quaternion(*quat_pos)
    
    point.pose = pose_list
    
    point.time_from_start = rospy.Duration(1)
    goal.trajectory.points.append(point)
    trajectory_client.send_goal(goal)

    trajectory_client.wait_for_result()




 

def main():

    rospy.init_node('Teleoperation_Position_Control',anonymous=True)
    print("main")

    while not rospy.is_shutdown():

        send_cartesian_trajectory()

    rospy.spin()




if __name__=='__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
       pass