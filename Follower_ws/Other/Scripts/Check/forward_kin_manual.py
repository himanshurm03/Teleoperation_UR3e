#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg  import JointState

def joint_callback(msg):
    
    
    joint_position =  msg.position 
    ####Number of links
    n = 6

    #####  Define DH parameters for UR3e
    # alp = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
    # a = [0, -0.425, -0.39225, 0, 0, 0]
    # d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]

    alp = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
    a = [0, -0.24355, -0.2132, 0, 0, 0]
    d = [0.15185, 0, 0, 0.13105, 0.08535, 0.0921]

   ############# d = [0.0892, 0, 0, 0.109, 0.095, 0.0823]
    th = np.deg2rad([60.80, -55.01, 60.47, -186.69, -0.40, 0.80])
    
    #th = ([joint_position[2], joint_position[1],joint_position[0],joint_position[3], joint_position[4], joint_position[5]])
    #print("joint_position: ",joint_position)
     

    ## Calibrated DH parameters
    # alp= np.deg2rad([89.852, 0.099, 0.1439, 89.763, -90.191, -0.192])
    # a=[0.000187, -0.423863, -0.391364, -0.000960, -0.000303, -0.001093];
    # d=[0.088852, 0, -0.000200 , 0.109956, 0.095160 , 0.081358];
    # th = np.deg2rad([180.00, -180.00, 90.0, -180.0, -90.0, 180.0])


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

    # if not singular:
    #     roll = np.arctan2(R_n_0[2, 1], R_n_0[2, 2])
    #     pitch = np.arctan2(-R_n_0[2, 0], sy)
    #     yaw = np.arctan2(R_n_0[1, 0], R_n_0[0, 0])
    # else:
    #     roll = np.arctan2(-R_n_0[1, 2], R_n_0[1, 1])
    #     pitch = np.arctan2(-R_n_0[2, 0], sy)
    #     yaw = 0

    # Display results
    print('T_6_0 is:')
    print(T_n_0)

    print('Position at end effector at (0,0,0.145):')
    print(p_E_0)

    print('Roll, Pitch, Yaw:')
    print('Roll:', roll)
    print('Pitch:', pitch)
    print('Yaw:', yaw)

def main():
    rospy.init_node('forwark_kinematics_UR5',anonymous=True)
    #joint_callback()
    rospy.Subscriber('/joint_states', JointState, joint_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
       pass