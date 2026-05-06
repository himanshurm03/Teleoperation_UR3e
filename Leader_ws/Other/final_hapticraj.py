#!/usr/bin/env python3

import rospy
import math
# from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniFeedback
from omni_msgs.msg import OmniState
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import String

from std_msgs.msg import Int32
import time 
import csv
from os import path
from std_msgs.msg import Float64MultiArray
from rospy import Time

# from std_msgs.msg import Time


#   For data collection defining global variable

global h_x, h_y, h_z, h_phi, h_th, h_psi
h_x, h_y, h_z, h_phi, h_th, h_psi = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

global h_q0, h_q1, h_q2, h_q3
h_q0, h_q1, h_q2, h_q3 = 0.0, 0.0, 0.0, 0.0


global th1_h, th2_h, th3_h, th4_h, th5_h, th6_h
th1_h, th2_h, th3_h, th4_h, th5_h, th6_h = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

global roll_h, pitch_h, yaw_h
roll_h, pitch_h, yaw_h = 0.0, 0.0, 0.0

global omni_feedback_msg
omni_feedback_msg = OmniFeedback()

alpha_0   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

global force
force = [0.0, 0.0, 0.0]

global Timee
Timee = [0.0, 0.0]

global Start_Timee
Start_Timee = [0.0, 0.0]

global first_string
first_string = "0"

global t_force_round
t_force_round = 0.0






vel_hap   = [0.0, 0.0, 0.0]
force     = [0.0, 0.0, 0.0]
#Po_1      = [0.0, 0.0, 0.0]
force_f   = [0.0, 0.0, 0.0]
## Time
js_start    = True
t_init      = 0
t_now       = 0
data_file   = "xyz"
elapsed_time   = 0




##########################################pub2 = rospy.Publisher('desired_pose',  Float32MultiArray, queue_size=10)



#

def hp_pose_callback(msg):
    global h_x, h_y, h_z, h_phi, h_th, h_psi, h_q0, h_q1, h_q2, h_q3

    h_x = msg.pose.position.x ## Findig the Pose
    h_y = msg.pose.position.y
    h_z = msg.pose.position.z

    h_q0 = msg.pose.orientation.w
    h_q1 = msg.pose.orientation.x
    h_q2 = msg.pose.orientation.y
    h_q3 = msg.pose.orientation.z

    #h_phi, h_th, h_psi = quat2eul(h_q0,h_q1,h_q2,h_q3)

    haptic_pose = [h_x, h_y, h_z, h_q0, h_q1, h_q2, h_q3]
    #print(haptic_pose)

   
    # return haptic_pose




def force_callback(mssg):
    global force, start_time, t_init, elapsed_time

    

    haptic_forcee = mssg.data 
    force = np.array(haptic_forcee)
    tine = round(time.time(),5)
    end_time = tine  - 1709846782.19786
    #print('force[4]',force[4])
   #elapsed_time =  force[3] + end_time - force[4]
    elapsed_time =   end_time - force[3]
    #print('pos',force[3],'roundtrip',elapsed_time)
    #print('force3',force[3],'endtime',end_time,'force4',force[4])
    #elapsed_times = elapsed_time.to_sec()
   # print(force[0], force[1], force[2]) 

    

   

    pub1 = rospy.Publisher('phantom/phantom/force_feedback', OmniFeedback, queue_size=10)

     # correct frames for device alignment
    fx, fy, fz = force[3], force[4], force[5]
    

    omni_feedback_msg.force.x = fx
    omni_feedback_msg.force.y = fy
    omni_feedback_msg.force.z = -fz
    omni_feedback_msg.position.x = force[0]
    omni_feedback_msg.position.y = force[1]
    omni_feedback_msg.position.z = force[2]

    pub1.publish(omni_feedback_msg)

    print('force from master',force)
# 

def time_callback(arg3):
    global first_string,t_force_round

    time_data = arg3.data
    separated_string = time_data.split(" ")
    first_string = separated_string[0]
    second_string = separated_string[1]
    t_force =  (rospy.get_time() - 1719404870.74)
   
  
    t_force_round = t_force - (float(second_string))
    #print('first string',first_string,'separated string',separated_string)
    print('first string',first_string,'separated string',separated_string,'roundtrip',t_force_round)

def hapticdata_publisher():
    pub2 = rospy.Publisher('haptic_out',  Float64MultiArray, queue_size=10)
    pub3 = rospy.Publisher('time_out', String , queue_size=10)
    
    
    rate = rospy.Rate(1000)
    
    
    # first_step = True

    while not rospy.is_shutdown():
        global  alpha_0, h_x, h_y, h_z, h_phi, h_th, h_psi, h_q0, h_q1, h_q2, h_q3, start_time, Start_Timee, js_start, force, data_file, t_now, t_init, elapsed_time,first_string,t_force_round  #data_file,force, alpha, force_f

       
        
        
        
        ####################alpha_0 = [x0, y0, z0, phi0, th0, psi0]
        #print(alpha_0)
        #start_time = rospy.get_time()
        #start_time = rospy.Time.now()
        alpha_0 = [h_x, h_y, h_z, h_q0, h_q1, h_q2, h_q3, 0.0]
        
        if (js_start==True):
            global t_init
            js_start = False
            t_init = time.time()

        t_current = time.time() - t_init
          


        #####################################################################################################
        if (t_current > 0): 
        #     alp_0        = str(alpha[0]) + ',' + str(alpha[1]) + ',' + str(alpha[2])
        #  #   E_in_2_0    = str(E_in_2[0]) + ',' + str(E_in_2[1]) + ',' + str(E_in_2[2])
        #  #   E_out_1_0   = str(E_out_1[0]) + ',' + str(E_out_1[1]) + ',' + str(E_out_1[2])
######raj
          if len(force)>=4:
         #data =  str(force[3]) + ',' + str(str(elapsed_time)) + '\n'
            data =  str(t_force_round) + ',' + '\n'
            file = open(data_file, 'a')
            file.write(data)
            file.close()
          ###else:   wireshark
           ##### print("not enough elements in force list")  wireshark
         #data =  str(force[3]) + ',' + '\n'
          


        #####################################################################################################   

       
       ############################### desired_pose = Float32MultiArray()
        hhaptic_pose = Float64MultiArray()
        Time_pose = Float64MultiArray()
        #message_energy.data = E_in_1
       ################################# desired_pose.data = alpha_0
        ###########3hhaptic_pose.data = alpha_0    ###raj
        #Start_Timee= [start_time, 0.0]
        #Start_Timee = [start_time.to_sec(), 0.0]

###############raaajjj###########
        # if(js_start==True):
        #     #global js_start
        #     global t_init
        #     js_start = False
        #     t_init = time.time()
        
        #t_now = rospy.get_time() - 1701889834.361106)
        hello_str = "%s" % (rospy.get_time() - 1719404870.74)
        #hello_str = "hello world %s" % 1700597277.361933
        hello_str += " " + first_string
        #hello_str =  rospy.get_time()
        ###rospy.loginfo(hello_str)  #######wireshark
        #t_now =time.time()-1700597277.361933
        #print('t_now',t_now)
        alpha_0[7]= t_now
        hhaptic_pose.data = alpha_0 
###########raajjj########################
      #  print(hhaptic_pose)

        Start_Timee = [t_now, 0.0]
        #print('start time', Start_Timee[0])

       

        #Time_pose.data = Start_Timee
       
        ###########################################pub2.publish(desired_pose)
        pub2.publish(hhaptic_pose)
        #pub3.publish(Time_pose)
        pub3.publish(hello_str)####wireshark 
        #pub3.publish(start_time)
        
        # pub3.publish(current_time)
        # pub4.publish(omni_feedback_msg)
        rate.sleep()



def main():
    secs = time.time()
    tt = time.localtime(secs)
    t = time.asctime(tt)

##raj
    global data_file
    file_name = 'hoip_haptic_'  + str(t) + '.csv'
    save_path = '/Desktop'
    data_file =  path.join(save_path, file_name)
    file = open(data_file,'w')
    file.close()
    time.sleep(2)


    # global alpha_0
    rospy.init_node("haptic_data",anonymous=True)
    rospy.Subscriber('phantom/phantom/pose', PoseStamped, hp_pose_callback)
    rospy.Subscriber("time_out_slave", String,time_callback)
    rospy.Subscriber("force_pub", Float64MultiArray, force_callback)
    
    
    
    hapticdata_publisher()
    
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


