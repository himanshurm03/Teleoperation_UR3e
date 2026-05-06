#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped,TwistStamped,WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math
import time
from tf2_msgs.msg import TFMessage
import pandas as pd
from omni_msgs.msg import OmniState,OmniFeedback
import os



# Initialize a DataFrame to store the end-effector velocity data
global df
df = pd.DataFrame(columns=["timestamp", "x_haptic", "y_haptic", "z_haptic","phi_haptic", "theta_haptic", "psi_haptic","x_robot","y_robot","z_robot","phi_robot","theta_robot","psi_robot","x_robot_desired","y_robot_desired","z_robot_desired","phi_robot_desired","theta_robot_desired","psi_robot_desired"])

global teleop_file_ur3e
teleop_file_ur3e = "Teleoperation file"

#############  Publishing velocity
global pub, pub_force
pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
pub_force =  rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)



############  global variables  ###################

global haptic_stylus_position, h_offset_array, haptic_angular , haptic_stylus_orientation

global th1_h, th2_h, th3_h, th4_h, th5_h, th6_h, th1_h_off,th2_h_off, th3_h_off, th4_h_off,th5_h_off,th6_h_off,t_init,t_now

global  end_effector_start,robot_ee_pose , end_effetor_desired_robot, q_dot_unfiltered

global buffer_quat,buffer_dt,buffer_ang, pub_dt,angular_velocities_calculated

global flag_offset,flag_pos_offset,ee_start,js_start,flag2

global robot_ext_force, buffer_force_x, buffer_force_y, buffer_force_z, unfiltered_robot_force, buffer_force_size,force_sensor_flag, force_send_on_haptic
global force_z_offset

global data_minus_13,data_minus_23,data_minus_33,data_minus_43,data_minus_63

haptic_stylus_position = np.empty(6)
haptic_angular = np.empty(3)
h_offset_array =np.empty(7)

buffer_dt = []
buffer_quat = []
buffer_ang = []
pub_dt = 0.0
t_past = 0.0
angular_velocities_calculated = []

end_effector_start = np.empty(6)
robot_ee_pose = np.empty(6)
end_effetor_desired_robot = np.empty(6)
haptic_stylus_orientation = np.empty(4)
q_dot_unfiltered = np.array([0,0,0,0,0,0])


[th1_h, th2_h, th3_h, th4_h, th5_h, th6_h, th1_h_off,th2_h_off, th3_h_off, th4_h_off,th5_h_off,th6_h_off] = [0,0,0,0,0,0,0,0,0,0,0,0]
t_now = 0.0
t_init = 0.0


flag_offset = True
flag_pos_offset = True
ee_start = True
js_start = True
flag2 = True
force_sensor_flag = True

robot_ext_force = np.empty(3)
unfiltered_robot_force = np.empty(3)
buffer_force_size = 100
buffer_force_x = [[]for _ in range(buffer_force_size)]
buffer_force_y = [[]for _ in range(buffer_force_size)]
buffer_force_z = [[]for _ in range(buffer_force_size)]
force_send_on_haptic = np.empty(3)
force_z_offset = 0.0

data_minus_13 = np.empty(6)
data_minus_23 = np.empty(6)
data_minus_33 = np.empty(6)
data_minus_43 = np.empty(6)
data_minus_53 = np.empty(6)
data_minus_63 = np.empty(6)


#########################################

def limit(x, x_min, x_max):
    return np.where(x < x_min, x_min, np.where(x > x_max, x_max, x))

def haptic_force(x_p,y_p,z_p):

    global robot_ext_force, buffer_force_x, buffer_force_y, buffer_force_z, force_send_on_haptic 

    global pub_force

    omni_feedback_msg = OmniFeedback() 

    buffer_force_size = 100
    d = 0.145   ############# attachment length
         
      
    # # correct frames for device alignment
    # fx, fy, fz = fy, fx, -fz                                    #######  ????

    fx, fy, fz = robot_ext_force[0], robot_ext_force[1], robot_ext_force[2]
    F = [fx, fy, fz]    
    
  
#####################################   IF BUFFER IS CREATED AND AVERAGE IS TAKEN 

    buffer_force_x.append(fx)
    buffer_force_y.append(fy)
    buffer_force_z.append(fz)

    if len(buffer_force_x) > buffer_force_size:
        del buffer_force_x[0]
    if len(buffer_force_y) > buffer_force_size:
        del buffer_force_y[0]
    if len(buffer_force_z) > buffer_force_size:
        del buffer_force_z[0]

    buffer_force_x_avg = np.median(buffer_force_x)
    buffer_force_y_avg = np.median(buffer_force_y)
    buffer_force_z_avg = np.median(buffer_force_z)

    
    force_send_on_haptic = np.array([buffer_force_x_avg,buffer_force_y_avg,buffer_force_z_avg])

    force_condition = np.sqrt(buffer_force_x_avg**2 + buffer_force_y_avg**2 + buffer_force_z_avg**2)
    
    print("force condition: ",force_condition)
    if force_condition > 0.25:
        
        omni_feedback_msg.position.x = x_p
        omni_feedback_msg.position.y = y_p
        omni_feedback_msg.position.z = z_p
        omni_feedback_msg.force.x =   buffer_force_x_avg
        omni_feedback_msg.force.y =   buffer_force_y_avg
        omni_feedback_msg.force.z =   buffer_force_z_avg
    else:
        omni_feedback_msg.position.x = x_p
        omni_feedback_msg.position.y = y_p
        omni_feedback_msg.position.z = z_p
        omni_feedback_msg.force.x = 0
        omni_feedback_msg.force.y = 0
        omni_feedback_msg.force.z = 0



    print('Omni_feedback_msg: ',omni_feedback_msg)
    pub_force.publish(omni_feedback_msg)


##########################################################raj###############################################

def force_callback(msg):
    
    global robot_ext_force, unfiltered_robot_force, force_sensor_flag, force_z_offset

    force_x = msg.wrench.force.x
    force_y = msg.wrench.force.y
    

    if force_sensor_flag:
        force_z_offset = msg.wrench.force.z
        force_sensor_flag = False


    force_z = msg.wrench.force.z - force_z_offset

    unfiltered_robot_force = np.array([force_x,force_y,force_z])

    for i in range(0, 3):
        unfiltered_robot_force[i] = limit(unfiltered_robot_force[i], -2, 2)

    robot_ext_force = moving_average_3(unfiltered_robot_force)

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

     
    rospy.init_node('Teleop_position_control',anonymous=True)

    

   #rate = rospy.Rate(1000)
    rospy.Subscriber('/phantom/phantom/pose', PoseStamped, haptic_callback)          # end effector position only, orientation wrong
    rospy.Subscriber('/phantom/state',OmniState, haptic_end_eff_velocity )   # end effector linear velocity only 
    rospy.Subscriber('/phantom/phantom/joint_states',JointState, haptic_offset)      # joint position 
    rospy.Subscriber('/tf',TFMessage,tf_callback)
    rospy.Subscriber('/joint_states', JointState, joint_callback)
    rospy.Subscriber('/ft_wrench',WrenchStamped,force_callback)


    save_interval = rospy.Duration(0.002)  # Save every 1 seconds
    rospy.Timer(save_interval, lambda event: save_data())
    
    
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass  


###########################################################################################################################################


#!/usr/bin/env python3

import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped,WrenchStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import math
import tf.transformations as transformations
from threading import Timer
import matplotlib.pyplot as plt
from omni_msgs.msg import OmniState,OmniFeedback
from collections import deque

class RobotEndEffectorController:

    def __init__(self):

        # Initializing node
        rospy.init_node('robot_end_effector_controller', anonymous=True)

        # Publishers
        self.force_pub =  rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)

        # Subscribers
        rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)

        # For initialization parameter
        self.start_time = None
        self.robot_force = np.zeros(3)
        self.haptic_force = np.zeros(3)

        # For plotting
        self.time_stamps = []
        self.robot_force_plot = []
        self.haptic_force_plot = []

        # For moving average filter
        self.haptic_window_size = 100
        self.force_window = deque(maxlen=self.haptic_window_size)

    def update_list(self):
        current_time = rospy.get_time()
        if self.start_time is None:
            self.start_time = current_time
        self.time_stamps.append(current_time - self.start_time)
        self.robot_force_plot.append(self.robot_force)
        self.haptic_force_plot.append(self.haptic_force)

    def robot_force_callback(self, msg: WrenchStamped):
        robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.force_window.append(robot_force)
        self.robot_force = np.mean(self.force_window, axis=0)

    def haptic_force_callback(self, event):
        force_pub_msg = OmniFeedback()
        force_pub_msg.force.x = self.robot_force[0]
        force_pub_msg.force.y = self.robot_force[1]
        force_pub_msg.force.z = self.robot_force[2]
        self.force_pub.publish(force_pub_msg)

        print("haptic_force_callback:", force_pub_msg.force)
        self.haptic_force = np.array([force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z])
        self.update_list()

    def plot_data(self):

        times = np.array(self.time_stamps)
        robot_force_plot = np.array(self.robot_force_plot)
        haptic_force_plot = np.array(self.haptic_force_plot)

        fig, axs = plt.subplots(3, 1, figsize=(18, 11))

        axs[0].plot(times, robot_force_plot[:,0], times, haptic_force_plot[:,0])
        axs[0].set_title('force in x direction')
        axs[0].legend(['robot', 'haptic'])

        axs[1].plot(times, robot_force_plot[:,1], times, haptic_force_plot[:,1])
        axs[1].set_title('force in y direction')
        axs[1].legend(['robot', 'haptic'])

        axs[2].plot(times, robot_force_plot[:,2], times, haptic_force_plot[:,2])
        axs[2].set_title('force in z direction')
        axs[2].legend(['robot', 'haptic'])
        
        plt.tight_layout()
        plt.show()

    def main_loop(self):
        rate = 500
        rospy.Timer(rospy.Duration(1.0/rate),self.haptic_force_callback)
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = RobotEndEffectorController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Plot the data after the ROS node is shutdown
        controller.plot_data() 

#############################################################################trial1##################################################

#!/usr/bin/env python3

import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped,WrenchStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import math
import tf.transformations as transformations
from threading import Timer
import matplotlib.pyplot as plt
from omni_msgs.msg import OmniState,OmniFeedback
from collections import deque

class RobotEndEffectorController:

    def __init__(self):

        # Initializing node
        rospy.init_node('robot_end_effector_controller', anonymous=True)

        # Publishers
        self.force_pub =  rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)

        # Subscribers
        rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)

        # For initialization parameter
        self.start_time = None
        self.robot_force = np.zeros(3)
        self.haptic_force = np.zeros(3)

        # For plotting
        self.time_stamps = []
        self.robot_force_plot = []
        self.haptic_force_plot = []

        # For moving average filter
        self.haptic_window_size = 100
        self.force_window = deque(maxlen=self.haptic_window_size)

        rospy.on_shutdown(self.shutdown_handler)

    def update_list(self):
        current_time = rospy.get_time()
        if self.start_time is None:
            self.start_time = current_time
        self.time_stamps.append(current_time - self.start_time)
        self.robot_force_plot.append(self.robot_force)
        self.haptic_force_plot.append(self.haptic_force)

    def robot_force_callback(self, msg: WrenchStamped):
        robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.force_window.append(robot_force)
        self.robot_force = np.mean(self.force_window, axis=0)

    def haptic_force_callback(self, event):
        force_pub_msg = OmniFeedback()
        force_pub_msg.force.x = self.robot_force[0]
        force_pub_msg.force.y = self.robot_force[1]
        force_pub_msg.force.z = self.robot_force[2]
        self.force_pub.publish(force_pub_msg)

        print("haptic_force_callback:", force_pub_msg.force)
        self.haptic_force = np.array([force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z])
        self.update_list()

    def plot_data(self):

        times = np.array(self.time_stamps)
        robot_force_plot = np.array(self.robot_force_plot)
        haptic_force_plot = np.array(self.haptic_force_plot)

        fig, axs = plt.subplots(3, 1, figsize=(18, 11))

        axs[0].plot(times, robot_force_plot[:,0], times, haptic_force_plot[:,0])
        axs[0].set_title('force in x direction')
        axs[0].legend(['robot', 'haptic'])

        axs[1].plot(times, robot_force_plot[:,1], times, haptic_force_plot[:,1])
        axs[1].set_title('force in y direction')
        axs[1].legend(['robot', 'haptic'])

        axs[2].plot(times, robot_force_plot[:,2], times, haptic_force_plot[:,2])
        axs[2].set_title('force in z direction')
        axs[2].legend(['robot', 'haptic'])
        
        plt.tight_layout()
        plt.show()     

    def shutdown_handler(self):

        self.ramp_down_force()
        rospy.loginfo("Shutting down...")
    
        # Send zero force command multiple times
        zero_force = OmniFeedback()
        zero_force.force.x = 0
        zero_force.force.y = 0
        zero_force.force.z = 0
        
        rate = rospy.Rate(100)  # 100 Hz
        for _ in range(50):  # Send zero force for 0.5 seconds
            self.force_pub.publish(zero_force)
            rate.sleep()
        
        rospy.loginfo("Zero force commands sent. Waiting for device to process...")
        rospy.sleep(1.0)  # Wait an additional second
        
        rospy.loginfo("Shutdown complete")

    def ramp_down_force(self):
        current_force = self.haptic_force.copy()
        rate = rospy.Rate(100)  # 100 Hz
        while not rospy.is_shutdown() and np.linalg.norm(current_force) > 0.01:
            current_force *= 0.95  # Reduce force by 5% each iteration
            force_msg = OmniFeedback()
            force_msg.force.x, force_msg.force.y, force_msg.force.z = current_force
            self.force_pub.publish(force_msg)
            rate.sleep()

    def main_loop(self):
        rate = 1000
        rospy.Timer(rospy.Duration(1.0/rate),self.haptic_force_callback)
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = RobotEndEffectorController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Main loop ended. Starting shutdown procedure...")
        rospy.sleep(1.0)  # Give some time for ROS to process
        controller.shutdown_handler()
        controller.plot_data() 

#########################################################################trial2#########################################################################

#!/usr/bin/env python3

import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import math
import tf.transformations as transformations
from threading import Timer
import matplotlib.pyplot as plt
from omni_msgs.msg import OmniState, OmniFeedback
from collections import deque
import time  # To add delay

class RobotEndEffectorController:

    def __init__(self):

        # Initializing node
        rospy.init_node('robot_end_effector_controller', anonymous=True)

        # Publishers
        self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)

        # Subscribers
        rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)

        # For initialization parameter
        self.start_time = None
        self.robot_force = np.zeros(3)
        self.haptic_force = np.zeros(3)

        # For plotting
        self.time_stamps = []
        self.robot_force_plot = []
        self.haptic_force_plot = []

        # For moving average filter
        self.haptic_window_size = 100
        self.force_window = deque(maxlen=self.haptic_window_size)

        # Shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        self.shutdown_flag = False  # To manage shutdown state

    def update_list(self):
        if self.shutdown_flag:
            return
        current_time = rospy.get_time()
        if self.start_time is None:
            self.start_time = current_time
        self.time_stamps.append(current_time - self.start_time)
        self.robot_force_plot.append(self.robot_force)
        self.haptic_force_plot.append(self.haptic_force)

    def robot_force_callback(self, msg: WrenchStamped):
        robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y+0.375, msg.wrench.force.z-0.189])
        self.force_window.append(robot_force)
        self.robot_force = np.mean(self.force_window, axis=0)

    def haptic_force_callback(self, event):
        if self.shutdown_flag:
            return
        force_pub_msg = OmniFeedback()
        
        # trial 1
        # force_pub_msg.force.x = self.robot_force[0]
        # force_pub_msg.force.y = self.robot_force[1]
        # force_pub_msg.force.z = -self.robot_force[2]

        # trial 2
        # force_pub_msg.force.x = self.robot_force[1]*0.766 + self.robot_force[0]*0.642
        # force_pub_msg.force.y = -self.robot_force[1]*0.642 + self.robot_force[0]*0.766
        # force_pub_msg.force.z = -self.robot_force[2]

        # trial 3 (working fine for wrist 3 angle 326.22)
        # force_pub_msg.force.x = self.robot_force[1]*0.642 - self.robot_force[0]*0.766
        # force_pub_msg.force.y = self.robot_force[1]*0.766 + self.robot_force[0]*0.642
        # force_pub_msg.force.z = -self.robot_force[2]

        # trial 4
        force_pub_msg.force.x = 0
        force_pub_msg.force.y = 0
        force_pub_msg.force.z = 0

        self.force_pub.publish(force_pub_msg)

        print("robot_force", self.robot_force)
        print("haptic_force:", force_pub_msg.force)
        self.haptic_force = np.array([force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z])
        self.update_list()

    def plot_data(self):

        times = np.array(self.time_stamps)
        robot_force_plot = np.array(self.robot_force_plot)
        haptic_force_plot = np.array(self.haptic_force_plot)

        min_len = min(len(times), len(robot_force_plot), len(haptic_force_plot))
        times = times[:min_len]
        robot_force_plot = robot_force_plot[:min_len]
        haptic_force_plot = haptic_force_plot[:min_len]

        fig, axs = plt.subplots(3, 1, figsize=(18, 11))

        axs[0].plot(times, robot_force_plot[:, 0], times, haptic_force_plot[:, 0])
        axs[0].set_title('Force in x direction')
        axs[0].legend(['Robot', 'Haptic'])

        axs[1].plot(times, robot_force_plot[:, 1], times, haptic_force_plot[:, 1])
        axs[1].set_title('Force in y direction')
        axs[1].legend(['Robot', 'Haptic'])

        axs[2].plot(times, robot_force_plot[:, 2], times, haptic_force_plot[:, 2])
        axs[2].set_title('Force in z direction')
        axs[2].legend(['Robot', 'Haptic'])
        
        plt.tight_layout()
        plt.show()

    def main_loop(self):
        rate = 500
        rospy.Timer(rospy.Duration(1.0 / rate), self.haptic_force_callback)
        rospy.spin()

    def shutdown_hook(self):
        self.shutdown_flag = True
        # Send zero force to the haptic device
        zero_force_msg = OmniFeedback()
        zero_force_msg.force.x = 0
        zero_force_msg.force.y = 0
        zero_force_msg.force.z = 0

        rospy.loginfo("Sending zero force to haptic device.")
        for _ in range(10):  # Send the message multiple times to ensure it gets through
            self.force_pub.publish(zero_force_msg)
            time.sleep(0.1)

        rospy.loginfo("Shutting down, sent zero force to haptic device.")
        
        # Plot the data after the ROS node is shut down
        self.plot_data()

if __name__ == "__main__":
    try:
        controller = RobotEndEffectorController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass