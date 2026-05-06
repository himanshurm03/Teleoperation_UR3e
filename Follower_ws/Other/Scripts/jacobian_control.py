#!/usr/bin/env python3


import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from cartesian_control_msgs.msg import FollowCartesianTrajectoryActionFeedback
from tf2_msgs.msg import TFMessage
import time


global pub,js_start, t_now,t_init,end_eff
pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)

js_start = True
t_now = 0.0
t_init = 0.0

end_eff = np.empty(3)
global flag,flag2,start_pos,end_pos
flag = True
flag2 = False
start_pos = np.empty(3)
end_pos = np.empty(3)



def end_eff_cb(msg):

    global end_eff_vel,df,  start_time 
    
    X_vel_ee = msg.feedback.actual.twist.linear.x
    Y_vel_ee = msg.feedback.actual.twist.linear.y
    Z_vel_ee = msg.feedback.actual.twist.linear.z

    X_angvel_ee = msg.feedback.actual.twist.angular.x
    Y_angvel_ee = msg.feedback.actual.twist.angular.y
    Z_angvel_ee = msg.feedback.actual.twist.angular.z

    print("X_vel_lin: ",X_vel_ee)
    print("X_vel_ang: ",X_angvel_ee)

def tf_callback(msg):
    global end_eff
    Xee = msg.transforms[0].transform.translation.x
    Yee = msg.transforms[0].transform.translation.y
    Zee = msg.transforms[0].transform.translation.z
    end_eff = np.array([Xee, Yee, Zee])
    print(end_eff)

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
    # print("Jacobian Matrix:")
    # print(J)


    send_joint_vel()
    

   # return J, p_E_0

def send_joint_vel():

    global pub,J,js_start,t_now, t_init
    global end_eff,flag,start_pos,flag2,end_pos

    omega = 2*np.pi*1
    oscillating = np.sin(omega*t_now)
    end_effector_vel = np.array([0.00,0.00,0.00,0.0,0.0,0.1])
    q_dot = np.matmul(np.linalg.pinv(J),end_effector_vel)
    
    velocity = Float64MultiArray()
    velocity.data = q_dot
    


    if js_start:
        js_start = False
        t_init = time.time()

    t_now = time.time() - t_init
    print(t_now)
 
    if t_now  < 6:
        pub.publish(velocity)
        if flag == True:
            start_pos = end_eff
            flag = False
    else:
        velocity2 = Float64MultiArray(data=[0.0,-0.0,0.0,0.0,0.0,0.0])
        pub.publish(velocity2)
        #if flag2 == True:
        end_pos = end_eff
        total_dist = end_pos - start_pos
        print("total_dist: ", total_dist)
        #    flag2 = False

    print(q_dot)



def main():
    rospy.init_node('velocity_control', anonymous=True)
    rospy.Subscriber('/pose_based_cartesian_traj_controller/follow_cartesian_trajectory/feedback',FollowCartesianTrajectoryActionFeedback,end_eff_cb)
    rospy.Subscriber('/joint_states', JointState, joint_callback)
    rospy.Subscriber('/tf', TFMessage, tf_callback)
    print("main")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass    