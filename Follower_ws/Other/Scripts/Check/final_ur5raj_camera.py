#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import WrenchStamped
import os.path
import time
from rospy import Time
from os import path
from geometry_msgs.msg import WrenchStamped

global tool_pos_linear
tool_pos_linear = [0.0, 0.0, 0.0]

global tool_pos_angular
tool_pos_angular = [0.0, 0.0, 0.0, 0.0]

global alpha
alpha = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

global alpha_0
alpha_0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

global alphaa_0
alphaa_0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

global diff_alpha
diff_alpha = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]

global dif_alpha
dif_alpha = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]

global cur_joint_pos
cur_joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

global sub_time
sub_time = [0.0, 0.0]

global Timee
Timee = [0, 0]

global pos_oneway
pos_oneway = 0.0

global round_way
round_way = 0.0

global force_real
force_real = [0.0,0.0,0.0,0.0]

global first_string

first_string='0'

f_t_d = 0.0

global t_ur5

global des_joint_pos
global des_joint_vel

des_joint_pos = [2.3968450477696024e-05, -5.4661427633106996e-05, -1.5707700888263147, -0.7854984442340296, -1.57084829012026, 1.5708271265029907] # this is the home location
des_joint_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

beta        = [0.0, 0.0, 0.0]
force       = [0.0, 0.0, 0.0, 0.0, 0.0]
force0      = [0.0, 0.0, 0.0]
Po_2        = [0.0, 0.0, 0.0]
E_in_2      = [0.0, 0.0, 0.0]
E_out_2     = [0.0, 0.0, 0.0]
step_diff   = [0.0, 0.0, 0.0]
step_diff_0 = [0.0, 0.0, 0.0]
E_in_1      = [0.0, 0.0, 0.0]
alpha_00    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
force_real = [0.0,0.0,0.0,0.0]
## Time
js_start    = True
t_init      = 0
t_now       = 0
t_now_1     = 0
t_ur5 = '0'

data_file ='xyz'




global force_file_name
force_file_name = "xyz"



global haptic_file_name
haptic_file_name = "xyz"

global joint_traj_msg
joint_traj_msg = JointTrajectory()

def limit(x,x_min,x_max):
    if(x<x_min):
        return x_min
    elif(x>x_max):
        return x_max
    else:
        return x


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


def haptic_callback(msg4): # Desired pose from haptic device
    global alphaa_0, force, start_time_1, js_start, pos_oneway, t_ur5, t_init,f_t_d

    #start_time_1= rospy.get_time()

    # if(js_start==True):
    #         #global js_start
    #         global t_init
    #         js_start = False
    #         t_init = time.time()
        
    #t_ur5 = rospy.get_time() - 1702555766.3387117
    #print('t_ur5',t_ur5)

    haptic_data = msg4.data
    alphaa_0 = np.array(haptic_data)

   
    


    #pos_oneway = start_time_1 - alphaa_0[7]
    
    
    #print(f"oneway: {pos_oneway} subscribed_time: {alphaa_0[7]}")
    #print('pos_oneway',pos_oneway)
    #print('subscribed',alphaa_0[7])

    

   
   # print(alphaa_0[0],alphaa_0[1],alphaa_0[2])
    if alphaa_0[0] > 0: 
       force[0] = 0.5
    else:
       force[0]=0 

    if alphaa_0[1] > 0: 
       force[1] = 0.5
    else:
       force[1]=0    

    if alphaa_0[2] > 0: 
       force[2] = 0.5
    else:
       force[2]=0     



def force_calculation(arg1):
    global force_real

    force_real_x = arg1.wrench.force.x
    force_real_y = arg1.wrench.force.y
    force_real_z = arg1.wrench.force.z
    
    force_real = [force_real_x,force_real_y,force_real_z,0.0]
    

    

    

def calc_jacob(q): ## Calcualtion of jacobian
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5]

    jacobian = np.array([[ (2183*np.cos(q1))/20000 + (823*np.cos(q1)*np.cos(q5))/10000 + (17*np.cos(q2)*np.sin(q1))/40 - (1569*np.sin(q1)*np.sin(q2)*np.sin(q3))/4000 + (823*np.cos(q2 + q3 + q4)*np.sin(q1)*np.sin(q5))/10000 - (591*np.cos(q2 + q3)*np.sin(q1)*np.sin(q4))/6250 - (591*np.sin(q2 + q3)*np.cos(q4)*np.sin(q1))/6250 + (1569*np.cos(q2)*np.cos(q3)*np.sin(q1))/4000, np.cos(q1)*((1569*np.sin(q2 + q3))/4000 + (17*np.sin(q2))/40 + np.sin(q5)*((823*np.cos(q2 + q3)*np.sin(q4))/10000 + (823*np.sin(q2 + q3)*np.cos(q4))/10000) + (591*np.cos(q2 + q3)*np.cos(q4))/6250 - (591*np.sin(q2 + q3)*np.sin(q4))/6250),                         np.cos(q1)*((591*np.cos(q2 + q3 + q4))/6250 + (1569*np.sin(q2 + q3))/4000 + (823*np.sin(q2 + q3 + q4)*np.sin(q5))/10000),                         np.cos(q1)*((591*np.cos(q2 + q3 + q4))/6250 + (823*np.sin(q2 + q3 + q4)*np.sin(q5))/10000), (823*np.cos(q1)*np.cos(q2)*np.cos(q5)*np.sin(q3)*np.sin(q4))/10000 - (823*np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q5))/10000 - (823*np.sin(q1)*np.sin(q5))/10000 + (823*np.cos(q1)*np.cos(q3)*np.cos(q5)*np.sin(q2)*np.sin(q4))/10000 + (823*np.cos(q1)*np.cos(q4)*np.cos(q5)*np.sin(q2)*np.sin(q3))/10000, 0],
	[ (2183*np.sin(q1))/20000 - (17*np.cos(q1)*np.cos(q2))/40 + (823*np.cos(q5)*np.sin(q1))/10000 - (823*np.cos(q2 + q3 + q4)*np.cos(q1)*np.sin(q5))/10000 + (591*np.cos(q2 + q3)*np.cos(q1)*np.sin(q4))/6250 + (591*np.sin(q2 + q3)*np.cos(q1)*np.cos(q4))/6250 - (1569*np.cos(q1)*np.cos(q2)*np.cos(q3))/4000 + (1569*np.cos(q1)*np.sin(q2)*np.sin(q3))/4000, np.sin(q1)*((1569*np.sin(q2 + q3))/4000 + (17*np.sin(q2))/40 + np.sin(q5)*((823*np.cos(q2 + q3)*np.sin(q4))/10000 + (823*np.sin(q2 + q3)*np.cos(q4))/10000) + (591*np.cos(q2 + q3)*np.cos(q4))/6250 - (591*np.sin(q2 + q3)*np.sin(q4))/6250),                         np.sin(q1)*((591*np.cos(q2 + q3 + q4))/6250 + (1569*np.sin(q2 + q3))/4000 + (823*np.sin(q2 + q3 + q4)*np.sin(q5))/10000),                         np.sin(q1)*((591*np.cos(q2 + q3 + q4))/6250 + (823*np.sin(q2 + q3 + q4)*np.sin(q5))/10000), (823*np.cos(q1)*np.sin(q5))/10000 - (823*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q5)*np.sin(q1))/10000 + (823*np.cos(q2)*np.cos(q5)*np.sin(q1)*np.sin(q3)*np.sin(q4))/10000 + (823*np.cos(q3)*np.cos(q5)*np.sin(q1)*np.sin(q2)*np.sin(q4))/10000 + (823*np.cos(q4)*np.cos(q5)*np.sin(q1)*np.sin(q2)*np.sin(q3))/10000,                      0],
	[                                                                                                                                                                                                                                                                                            0,                                                      (591*np.sin(q2 + q3 + q4))/6250 - (823*np.sin(q2 + q3 + q4 + q5))/20000 - (1569*np.cos(q2 + q3))/4000 - (17*np.cos(q2))/40 + (823*np.sin(q2 + q3 + q4 - q5))/20000, (591*np.sin(q2 + q3 + q4))/6250 - (823*np.sin(q2 + q3 + q4 + q5))/20000 - (1569*np.cos(q2 + q3))/4000 + (823*np.sin(q2 + q3 + q4 - q5))/20000, (591*np.sin(q2 + q3 + q4))/6250 - (823*np.sin(q2 + q3 + q4 + q5))/20000 + (823*np.sin(q2 + q3 + q4 - q5))/20000,                                                                                                                                                                           - (823*np.sin(q2 + q3 + q4 + q5))/20000 - (823*np.sin(q2 + q3 + q4 - q5))/20000,                                                     0],
	[                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                  np.sin(q1),                                                                                                                           np.sin(q1),                                                                                                np.sin(q1),                                                                                                                                                                                                                           np.sin(q2 + q3 + q4)*np.cos(q1),   np.cos(q5)*np.sin(q1) - np.cos(q2 + q3 + q4)*np.cos(q1)*np.sin(q5)],
	[0,                                                                                                                                                                                                 -np.cos(q1),                                                                                                                          -np.cos(q1),                                                                                               -np.cos(q1),                                                                                                                                                                                                                           np.sin(q2 + q3 + q4)*np.sin(q1), - np.cos(q1)*np.cos(q5) - np.cos(q2 + q3 + q4)*np.sin(q1)*np.sin(q5)],
	[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        1,                                                                                                                                                                                                        0,                                                                                                                                 0,                                                                                                      0,                                                                                           -np.cos(q2 + q3 + q4),                            -np.sin(q2 + q3 + q4)*np.sin(q5)]])
    return jacobian

def cal_K_alpha(k_p):  # calculation of propotional matrix K_p
    K_alpha = np.array([[k_p[0], 0.0, 0.0, 0.0, 0.0, 0.0], 
                        [0.0, k_p[1], 0.0, 0.0, 0.0, 0.0,],
                        [0.0, 0.0, k_p[2], 0.0, 0.0, 0.0,], 
                        [0.0, 0.0, 0.0, k_p[3], 0.0, 0.0], 
                        [0.0, 0.0, 0.0, 0.0, k_p[4], 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, k_p[5]]])
    return K_alpha

def calc_M(phi, th, psi):   ## Calculation of mapping matrix
    m11 = 1.0
    m12 = np.sin(phi)*np.tan(th)
    m13 = np.cos(phi)*np.tan(th)
    m21 = 0.0
    m22 = np.cos(phi)
    m23 = -np.sin(phi)
    m31 = 0.0
    m32 = np.sin(phi)/np.cos(th)
    m33 = np.cos(phi)/np.cos(th)

    M = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 [0.0, 1.0, 0.0, 0.0, 0.0, 0.0,],
                 [0.0, 0.0, 1.0, 0.0, 0.0, 0.0,], 
                 [0.0, 0.0, 0.0, m11, m12, m13],
                 [0.0, 0.0, 0.0, m21, m22, m23],
                 [0.0, 0.0, 0.0, m31, m32, m33]])
    return M


def calc_q_dot(J,M,K_alpha,alpha_0,alpha):  ## Calcualtion of q_dot
    # print('0 ',alpha)
    # print('c ', alpha_0)
    #print(alpha[3]*180.0/math.pi)
    #print(manipulability(J))
    global diff_alpha
    for i in range(0,6):
        diff_alpha[i] = alpha_0[i] - alpha[i]
    # print(diff_alpha)ethdaq_data

    v_des = np.matmul(np.linalg.pinv(M),K_alpha.dot(np.array(diff_alpha)))
    # print(v_des[0],v_des[1],v_des[2],v_des[3],v_des[4],v_des[5])
    q_dot = np.matmul(np.matmul(np.linalg.pinv(J),np.linalg.pinv(M)),K_alpha).dot(np.array(diff_alpha))
    # print(q_dot)
    
   
    return v_des,q_dot

pub  = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=10)
pub1 = rospy.Publisher("mess_E_in_2",Float32MultiArray, queue_size=10)
#######################pub2 = rospy.Publisher("force_pub",Float32MultiArray, queue_size=10)


def quat2eul(q0,q1,q2,q3):
    phi = math.atan2(2*(q0*q1 + q2*q3),1-2*((q1)**2+(q2)**2))
    th  = math.asin(2*(q0*q2 - q3*q1))
    psi = math.atan2(2*(q0*q3 + q1*q2),1-2*((q2)**2+(q3)**2))
    return phi, th, psi




# 
    


# def E_input_1(msg4):
#     global E_in_1
#     E_in_1 = msg4.data


def talker():
    first_step = True
    pub2 = rospy.Publisher("force_pub",Float32MultiArray, queue_size=10)######wireshark
    pub3 = rospy.Publisher("time_pub",Float32MultiArray, queue_size=10)
    pub4 = rospy.Publisher("time_out_slave",String,queue_size=10)


    print("talker")
    rate = rospy.Rate(1000)    ####raj



    # rospy.init_node('talker',anonymous=True)
    while not rospy.is_shutdown():
        #global des_joint_vel, alpha_0, force, Po_2, E_in_2, E_out_2, step_diff, alpha_00, cur_joint_pos, step_diff_0, js_start, t_init
        global des_joint_vel, alpha_0, force, step_diff, alpha_00, cur_joint_pos, step_diff_0, js_start, t_init, pos_oneway, t_now_1, t_ur5,first_string,force_real,data_file,round_way

        

        global alpha
        # print('alpha_c->',alpha)
        if(js_start==True):
            #global js_start
          global t_init
          js_start = False 
          t_init = time.time() 
        
        t_now = time.time() - t_init  
        

        #####################################print(t_now)
        # print(t_now, alpha_0)
        # print(t_now, alpha)
        
        #rate = rospy.Rate(1000)

        k_xyz = 2
        k_pts = 2
        ## ----------------------
        # print(t_now)

        ## ----------------------
        
        k_x   = k_xyz
        k_y   = k_xyz
        k_z   = k_xyz
        k_phi = k_pts
        k_th  = k_pts
        k_psi = k_pts
        k_p = [k_x, k_y, k_z, k_phi, k_th, k_psi]
        K_alpha = cal_K_alpha(k_p)
        #print(K_alpha)ethdaq_data
        #print("---")

        Mapping = calc_M(alpha[3], alpha[4], alpha[5])
        #print(M)
        #print("---")

        Jaco = calc_jacob(cur_joint_pos)
        #print(J)
        #print("---")
        # alpha_0 = get_des_pos_haptic()
        # #print(alpha_0_)
        # #F = []
        # F = force_feedback(alpha_0[0],alpha_0[1],alpha_0[2])
        # #print(F)
        v_des,q_dot = calc_q_dot(Jaco,Mapping,K_alpha,alpha_00,alpha)
        des_joint_vel = [0,0,0,0,0,0]
        des_joint_vel = q_dot

        # ####################################################
        if (t_now > 1.0):
            data_0      = str(round_way) + ',' + '\n'
            file = open(data_file,'a')
            file.write(data_0)
            file.close
        #     data_c      = str(alpha[0]) + ',' + str(alpha[1]) + ',' + str(alpha[2]) + ','  + str(alpha[3]) + ',' + str(alpha[4]) + ',' + str(alpha[5])
        #     force_0     = str(force[0]) + ',' + str(force[1]) + ',' + str(force[2])
        #     al_0        = str(beta[0]) + ',' + str(beta[1]) + ',' + str(beta[2])
        #     E_in_1_0    = str(E_in_1[0]) + ',' + str(E_in_1[1]) + ',' + str(E_in_1[2])
        #     E_out_2_0   = str(E_out_2[0]) + ',' + str(E_out_2[1]) + ',' + str(E_out_2[2])
            
        ####################################################
        w_max = 0.3 

        for i in range(0,6):
            des_joint_vel[i] = limit(des_joint_vel[i],-w_max,w_max)
        
        #rospy.loginfo(joint_traj_msg)
        #print("des_joint_vel", des_joint_vel)
        
        velocity_scale = 2.0
        for i in range(0,6):
            des_joint_vel[i] = des_joint_vel[i]*velocity_scale
        
        joint_traj_msg.points = [JointTrajectoryPoint(velocities=des_joint_vel)] 
        #pub.publish(joint_traj_msg)

        ################################################################


        mess_E_in_2 = Float32MultiArray()
        force_pub   = Float32MultiArray()
        #time_out = String()
        # time_pub = Int32MultiArray()
        ###############################start_time_2 = rospy.get_time()
        #Timee[0] =  start_time_2
        #Timee[1] =  pos_oneway

        
        t_now_1 = "%s" % (rospy.get_time() - 1710182175.92)

        
        # print(time.time())

        #force_real[3] = t_now_1
        
        #force[4] = t_now_1

        t_now_1 += " " + first_string
        ##########################force[4] = start_time_2
        ########################### force_pub.data = force_real
        force_pub.data = force
        #time_pub.data = Timee
        mess_E_in_2.data = E_in_2
        #pub1.publish(mess_E_in_2)
        pub2.publish(force_pub)
        pub4.publish(t_now_1)  
        #print(rospy.get_time())
        #ret, frame = cap.read()
        #if ret:
            #print('loop')
            #image_message = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            #image_pub.publish(image_message)
        
        ##############rospy.Subscriber("time_out",String,Time_callback)
        ################rospy.Subscriber("haptic_out",Float32MultiArray,haptic_callback)
        #print(pos_oneway)
        rate.sleep()
        #pub3.publish(time_pub)
        

def main():

    secs = time.time()
    tt = time.localtime(secs)
    t = time.asctime(tt)

    global data_file
    file_name = 'delay_h' + str(t) + '.csv'
    save_path = '/home/autonomous-lab/catkin_ws/src/service_pkg'
    data_file= path.join(save_path,file_name)
    file = open(data_file, 'w')
    file.close()
    time.sleep(2)


    rospy.init_node('task_space_traj_tracking', anonymous=True)
    #rospy.Subscriber('/joint_states', JointState, js_callback)
    #rospy.Subscriber('/tf', TFMessage , tf_callback)
    
    rospy.Subscriber("time_out",String,Time_callback)
    rospy.Subscriber("haptic_out",Float32MultiArray,haptic_callback)
    rospy.Subscriber("/ethdaq_data",WrenchStamped,force_calculation)          ############  change it to ATI Gamma sensor topic
   
    #rospy.Subscriber("/message_energy",Float32MultiArray,E_input_1)
    # rospy.Subscriber('/ethdaq_data', WrenchStamped, force_w)
    talker()
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
