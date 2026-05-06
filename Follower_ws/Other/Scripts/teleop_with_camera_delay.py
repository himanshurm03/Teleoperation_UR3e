#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped,TwistStamped  
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray,Float32MultiArray,String
import math
import time
from tf2_msgs.msg import TFMessage
import pandas as pd
from omni_msgs.msg import OmniState
import os



# Initialize a DataFrame to store the end-effector velocity data
global df
df = pd.DataFrame(columns=["timestamp", "x_haptic", "y_haptic", "z_haptic","phi_haptic", "theta_haptic", "psi_haptic","x_robot","y_robot","z_robot","phi_robot","theta_robot","psi_robot","x_robot_desired","y_robot_desired","z_robot_desired","phi_robot_desired","theta_robot_desired","psi_robot_desired"])

global teleop_file_ur3e
teleop_file_ur3e = "Teleoperation file"

#############  Publishing velocity and time delay   #######################
global pub, pub_delay
pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
pub_delay = rospy.Publisher("time_out_slave",String,queue_size=10)


############  global variables  ###################

global haptic_stylus_position, h_offset_array, haptic_angular , haptic_stylus_orientation

global th1_h, th2_h, th3_h, th4_h, th5_h, th6_h, th1_h_off,th2_h_off, th3_h_off, th4_h_off,th5_h_off,th6_h_off,t_init,t_now,first_string

global  end_effector_start,robot_ee_pose , end_effetor_desired_robot, q_dot_unfiltered


global flag_offset,flag_pos_offset,ee_start,js_start,flag2

haptic_stylus_position = np.empty(6)
haptic_angular = np.empty(3)
h_offset_array =np.empty(7)

end_effector_start = np.empty(6)
robot_ee_pose = np.empty(6)
end_effetor_desired_robot = np.empty(6)
haptic_stylus_orientation = np.empty(4)
q_dot_unfiltered = np.array([0,0,0,0,0,0])

[th1_h, th2_h, th3_h, th4_h, th5_h, th6_h, th1_h_off,th2_h_off, th3_h_off, th4_h_off,th5_h_off,th6_h_off] = [0,0,0,0,0,0,0,0,0,0,0,0]
t_now = 0.0
t_init = 0.0
first_string = '0'

flag_offset = True
flag_pos_offset = True
ee_start = True
js_start = True
flag2 = True

#########################################


################################  TIME DELAY CALCULATION
def Time_callback(msg5):
    global start_time_1, pos_oneway, sub_time, t_ur5, alphaa_0,f_t_d,first_string,round_way

    time_data = msg5.data
    separated_string = time_data.split(" ")
    first_string = separated_string[0]
    second_string = separated_string[1]
    #sub_time = np.array(time_data)
    

    

    ###t_ur5 = (rospy.get_time() - 1710182175.92)
    t_ur5 = (rospy.get_time() - 1710182175.92)
    #t_ur5 = rospy.get_time() - 1710703674.219843
    round_way =  ( t_ur5 - float(second_string))
    
    print('round_way',round_way,'t_ur5',t_ur5,'second_string',separated_string)

    
    #print('pos_oneway',pos_oneway)
    #print('t_ur5',t_ur5,'subscribed',sub_time[0])    

##############################




def limit(x, x_min, x_max):
    return np.where(x < x_min, x_min, np.where(x > x_max, x_max, x))

def dead_band_th(theta_array):
    for j in theta_array: 

        if abs(j) < 0.0014:
            j = 0.0

    return theta_array

def quat2euler(scalar_angle, vec_1, vec_2, vec_3):
    phi = math.atan2(2*(scalar_angle*vec_1 + vec_2*vec_3),1-2*((vec_1)**2+(vec_2)**2))
    th  = math.asin(2*(scalar_angle*vec_2 - vec_3*vec_1))
    psi = math.atan2(2*(scalar_angle*vec_3 + vec_1*vec_2),1-2*((vec_2)**2+(vec_3)**2))
    return phi, th, psi


def Qx(th):
    qx = np.matrix([[1, 0, 0],[0, math.cos(th), -math.sin(th)],[0, math.sin(th), math.cos(th)]])
    return qx

def Qy(th):
    qy = np.matrix([[math.cos(th), 0, math.sin(th)],[0, 1, 0],[-math.sin(th), 0, math.cos(th)]])
    return qy

def Qz(th):
    qz = np.matrix([[math.cos(th), -math.sin(th), 0],[math.sin(th), math.cos(th), 0], [0, 0, 1]])
    return qz

# def Q(th1, th2, th3, th4, th5, th6):
    

#     q1 = Qz(th1)
#     q2 = Qx(th2)
#     q3 = Qx(th3)
#     q4 = Qz(th4)
    
#     q5 = Qx(th5)

#     q6 = Qy(th6)

#     q = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(q1,q2),q3),q4),q5),q6)
#     return q

def Q(theta_q):
    


    q1 = Qz(theta_q[0])
    q2 = Qx(theta_q[1])
    q3 = Qx(theta_q[2])
    q4 = Qz(theta_q[3])
    
    q5 = Qx(theta_q[4])

    q6 = Qy(theta_q[5])

    q = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(q1,q2),q3),q4),q5),q6)
    return q





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

    haptic_stylus_orientation[0] = msg.pose.orientation.w 
    haptic_stylus_orientation[1] = msg.pose.orientation.x 
    haptic_stylus_orientation[2] = msg.pose.orientation.y 
    haptic_stylus_orientation[3] = msg.pose.orientation.z 

    # roll_haptic,pitch_haptic,yaw_haptic = quat2euler(haptic_stylus_orientation[0],haptic_stylus_orientation[1],haptic_stylus_orientation[2],haptic_stylus_orientation[3])
    # haptic_angular = np.array([roll_haptic,pitch_haptic,yaw_haptic])

    # haptic_angular = limit(haptic_angular, -0.2, 0.2)
    
    if abs(haptic_stylus_position[0]) < 10 or abs(haptic_stylus_position[1]) < 10 or abs(haptic_stylus_position[2]) < 10:    ###################  
        ''' This part is not working as we can see haptic stylus postion is giving high values along Z direction'''
        haptic_stylus_position = np.array([haptic_stylus_position[0],haptic_stylus_position[1],haptic_stylus_position[2],haptic_angular[0],haptic_angular[1],haptic_angular[2]])
    print("Haptic_stylus_position(in callback): ",haptic_stylus_position)



def haptic_end_eff_velocity(msg4):
    global hap_end_eff_vel
    hap_vel_const = 1000                                  #####conversion of haptic vel from mm/s to m/s

    hap_end_eff_x = msg4.velocity.x
    hap_end_eff_x *= 1

    hap_end_eff_y = msg4.velocity.y
    hap_end_eff_y *= 1
    hap_end_eff_z = msg4.velocity.z
    hap_end_eff_z *= 1


    hap_end_eff_vel = np.array([hap_end_eff_x,hap_end_eff_y,hap_end_eff_z])/hap_vel_const


def calc_M(phi, th, psi): 

     ## Mapping for angular values: ZYZ Euler set is used  

    #########   ZYX  Euler set  ##########

    # m11 = 1
    # m12 = 0
    # m13 = -np.sin(th)
    # m21 = 0
    # m22 = np.cos(phi)
    # m23 = np.sin(phi)*np.cos(th)
    # m31 = 0
    # m32 = -np.sin(phi)
    # m33 = np.cos(phi)*np.cos(th) 

    #####33333333333333    M inverse    ########################

    m11 = 1
    m12 = 0
    m13 = np.sin(th)
    m21 = 0
    m22 = np.cos(phi)
    m23 = -np.sin(phi)*np.cos(th)
    m31 = 0
    m32 = np.sin(phi)
    m33 = np.cos(phi)*np.cos(th)         


    M = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0,],
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0,], 
        [0.0, 0.0, 0.0, m11, m12, m13], 
        [0.0, 0.0, 0.0, m21, m22, m23],
        [0.0, 0.0, 0.0, m31, m32, m33]])

    return M


def cal_K_alpha(k_p):
    K_alpha = np.array([[k_p[0], 0.0, 0.0, 0.0, 0.0, 0.0], 
                        [0.0, k_p[1], 0.0, 0.0, 0.0, 0.0,],
                        [0.0, 0.0, k_p[2], 0.0, 0.0, 0.0,], 
                        [0.0, 0.0, 0.0, k_p[3], 0.0, 0.0], 
                        [0.0, 0.0, 0.0, 0.0, k_p[4], 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, k_p[5]]])
    return K_alpha



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
    
    

    roll_haptic,pitch_haptic,yaw_haptic = quat2euler(q0_h,q1_h,q2_h,q3_h)

    
    
    haptic_angular = np.array([roll_haptic,pitch_haptic,yaw_haptic])
    haptic_angular = limit(haptic_angular, -0.1, 0.1)
    
    print("Haptic angular: ",haptic_angular)
 






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

    
    roll_robot,pitch_robot,yaw_robot = quat2euler(W_ori_ee,X_ori_ee,Y_ori_ee,Z_ori_ee)


    
    if X_pos_ee != -0.24355:
        robot_ee_pose = np.array([X_pos_ee,Y_pos_ee,Z_pos_ee,roll_robot,pitch_robot,yaw_robot ])

    # if robot_ee_pose[0] - robot_offset[0] != 0.0:
    if ee_start == True:
        end_effector_start = robot_ee_pose
        ee_start = False

        print("End_effector_start; ",end_effector_start)



def desired_traj(xh,yh,zh,roll_h,pitch_h,yaw_h):
    
    """ Set your goal position at robot end and not on haptic device end.
    \n roll_h,pitch_h,yaw_h refers to angular position of haptic end effector. """


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

        roll_desired = roll_h + end_effector_start[3]
        pitch_desired = pitch_h + end_effector_start[4] 
        yaw_desired = yaw_h + end_effector_start[5]
        


        print(f"z_0: {z_0}")
        print(f"zh: {zh}")
        print(f"yaw_desired: {yaw_desired}")

    
    
    des_traj_array = np.array([x_0,y_0,z_0,roll_desired,pitch_desired,yaw_desired])
    print("desired traj: ", des_traj_array)
    return des_traj_array




def joint_callback(msg):
    global linear_pos, quaternion,J,joint_position

    joint_position = msg.position

    n = 6
    alp = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
    a = [0, -0.24355, -0.2132, 0, 0, 0]
    d = [0.15185, 0, 0, 0.13105, 0.08535, 0.0921]
    #th = np.deg2rad([9.80, -225.01, 107.47, -172.69, -100.40, 270.80])

    th = ([joint_position[2], joint_position[1],joint_position[0],joint_position[3], joint_position[4], joint_position[5]])

    d_E_n = np.array([0, 0, 0.145])
    T_i = np.eye(4)
    Ts = [T_i]  # Store transformation matrices

    for i in range(n):
        T_i_i_m = np.array([
            [np.cos(th[i]), -np.sin(th[i]) * np.cos(alp[i]), np.sin(th[i]) * np.sin(alp[i]), a[i] * np.cos(th[i])],
            [np.sin(th[i]), np.cos(th[i]) * np.cos(alp[i]), -np.cos(th[i]) * np.sin(alp[i]), a[i] * np.sin(th[i])],
            [0, np.sin(alp[i]), np.cos(alp[i]), d[i]],
            [0, 0, 0, 1]
        ])
        T_i_0 = T_i @ T_i_i_m
        Ts.append(T_i_0)  # Append transformation matrix to list
        T_i = T_i_0

    T_n_0 = T_i_0
    P_E_0 = T_n_0 @ np.append(d_E_n, 1)
    p_E_0 = P_E_0[:3]

    # Jacobian Calculation
    J = np.zeros((6, n))
    for i in range(n):
        T_i_0 = Ts[i]
        R_i_0 = T_i_0[:3, :3]
        p_i_0 = T_i_0[:3, 3]

        # z-axis of each joint in base frame
        z_i_0 = R_i_0[:, 2]

        # Linear velocity part
        J_p = np.cross(z_i_0, (p_E_0 - p_i_0))

        # Angular velocity part
        J_o = z_i_0

        # Assemble Jacobian
        J[:3, i] = J_p
        J[3:, i] = J_o

    # print("Transformation matrix: ")
    # print(T_n_0)
    print("Jacobian Matrix:")
    print(J)


    send_joint_vel()

def send_joint_vel():

    global pub,J,js_start,t_now, t_init , haptic_stylus_position,robot_ee_pose ,end_effetor_desired_robot, q_dot_unfiltered
    global rate, teleop_file_ur3e, pub_delay, first_string


    if js_start:
        js_start = False
        t_init = time.time()

    t_now = time.time() - t_init
    print(t_now)
    
    

    t_now_1 = "%s" % (rospy.get_time() - 1710182175.92)
    t_now_1 += " " + first_string












    
    end_effetor_desired_robot = desired_traj(haptic_stylus_position[0],haptic_stylus_position[1],haptic_stylus_position[2],haptic_stylus_position[3],haptic_stylus_position[4],haptic_stylus_position[5])
    

    mapping_call = calc_M(haptic_angular[0],haptic_angular[1],haptic_angular[2])
    k_array = np.array([4,4,4,0.0,0.0,0.0])
    #k_array = np.array([4,4,4,4,4,4])
    Kcall = cal_K_alpha(k_array)
    
    
    diff_position = end_effetor_desired_robot - robot_ee_pose
 

    if np.sqrt((end_effetor_desired_robot[3] - robot_ee_pose[3])**2 + (end_effetor_desired_robot[4] - robot_ee_pose[4])**2 + (end_effetor_desired_robot[5] - robot_ee_pose[5])**2) > 0.02:  

        #q_dot_unfiltered = np.matmul(np.linalg.pinv(J),np.matmul(Kcall,np.matmul(np.linalg.inv(mapping_call),diff_position)))
        q_dot_unfiltered = np.matmul(np.linalg.pinv(J),np.matmul(Kcall,np.matmul(mapping_call,diff_position)))
    else:
        q_dot_unfiltered = np.array([0,0,0,0,0,0])
    



    
    if t_now > 0.0 :
        data_1 = str(t_now) + ',' + str(q_dot_unfiltered[0]) + ',' + str(q_dot_unfiltered[1]) + ',' + str(q_dot_unfiltered[2]) + ',' + str(q_dot_unfiltered[3]) + ',' + str(q_dot_unfiltered[4]) + ',' + str(q_dot_unfiltered[5]) 
        data_2 = str(haptic_stylus_position[0]) + ',' + str(haptic_stylus_position[1]) + ',' + str(haptic_stylus_position[2]) + ',' + str(haptic_stylus_position[3]) + ',' + str(haptic_stylus_position[4]) + ',' + str(haptic_stylus_position[5]) 
        data_3 = str(robot_ee_pose[0]) + ',' + str(robot_ee_pose[1]) + ',' + str(robot_ee_pose[2]) + ',' + str(robot_ee_pose[3]) + ',' + str(robot_ee_pose[4]) + ',' + str(robot_ee_pose[5]) 
        data_4 = str(end_effetor_desired_robot[0]) + ',' + str(end_effetor_desired_robot[1]) + ',' + str(end_effetor_desired_robot[2]) + ',' + str(end_effetor_desired_robot[3]) + ',' + str(end_effetor_desired_robot[4]) + ',' + str(end_effetor_desired_robot[5]) 
        data_5 = data_1 + ',' + data_2 + ',' + data_3 + ',' + data_4 + ',' + '\n'
        
        file = open(teleop_file_ur3e, 'a')   #a = append
        file.write(data_5)
        file.close()
       
    
    
    
    
    
    
    
    
    
    max_rotational_vel = 0.49
    q_dot_unfiltered = limit(q_dot_unfiltered, -max_rotational_vel, max_rotational_vel)

    velocity_scale = 1.0
    q_dot_unfiltered = q_dot_unfiltered * velocity_scale
    
    
    
    velocity = Float64MultiArray()
    velocity.data = q_dot_unfiltered
    print("q_dot: " ,q_dot_unfiltered)





    if (q_dot_unfiltered[0]) < 1 and (q_dot_unfiltered[1]) < 1 and (q_dot_unfiltered[2]) < 1 and (q_dot_unfiltered[3]) < 1 and (q_dot_unfiltered[4]) < 1 and (q_dot_unfiltered[5]) < 1:
        pub.publish(velocity)
        pub_delay.publish(t_now_1)  


    # Append the data to the DataFrame
    global df
    new_data = pd.DataFrame({
        "timestamp": [t_now],
        "x_haptic": [haptic_stylus_position[0]],
        "y_haptic": [haptic_stylus_position[1]],
        "z_haptic": [haptic_stylus_position[2]],
        "phi_haptic": [haptic_stylus_position[3]],
        "theta_haptic": [haptic_stylus_position[4]],
        "psi_haptic": [haptic_stylus_position[5]],
        "x_robot" : [robot_ee_pose[0]] ,
        "y_robot" : [robot_ee_pose[1]] ,
        "z_robot" : [robot_ee_pose[2]] ,
        "phi_robot" : [robot_ee_pose[3]] ,
        "theta_robot" : [robot_ee_pose[4]] ,
        "psi_robot" : [robot_ee_pose[5]] ,
        "x_robot_desired" : [end_effetor_desired_robot[0]] ,
        "y_robot_desired" : [end_effetor_desired_robot[1]] ,
        "z_robot_desired" : [end_effetor_desired_robot[2]] ,
        "phi_robot_desired" : [end_effetor_desired_robot[3]] ,
        "theta_robot_desired" : [end_effetor_desired_robot[4]] ,
        "psi_robot_desired" : [end_effetor_desired_robot[5]] ,
 
    })

  



    df = pd.concat([df, new_data], ignore_index=True)
    # rate.sleep()

    


    

def save_data():
    global df , t_now
    
    if t_now > 3:
        save_path = '/home/user/catkin_UR_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/data_files/files_through_os_module'
        df.to_csv(save_path, index=False)





def main():
    global rate

    secs = time.time()
    tt = time.localtime(secs)
    t = time.asctime(tt)
    global teleop_file_ur3e
    file_name = 'teleop_vel_control'  + str(t) + '.csv'
    save_path_os = '/home/user/catkin_UR_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/data_files/files_through_os_module'
    teleop_file_ur3e = os.path.join(save_path_os, file_name)
    file = open(teleop_file_ur3e,'w')
    file.close()

    # rate = rospy.Rate(500) 
    rospy.init_node('Teleop_position_control',anonymous=True)


    rospy.Subscriber("time_out",String,Time_callback)
    #rospy.Subscriber("haptic_out",Float32MultiArray,haptic_callback)
    rospy.Subscriber('/phantom/pose', PoseStamped, haptic_callback)          # end effector position only, orientation wrong
    rospy.Subscriber('/phantom/state',OmniState, haptic_end_eff_velocity )   # end effector linear velocity only 
    rospy.Subscriber('/phantom/joint_states',JointState, haptic_offset)      # joint position 
    rospy.Subscriber('/tf',TFMessage,tf_callback)
    rospy.Subscriber('/joint_states', JointState, joint_callback)


    save_interval = rospy.Duration(0.002)  # Save every 1 seconds
    rospy.Timer(save_interval, lambda event: save_data())

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass    