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
        rate = 1000
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














# if force_sensor_flag:
        #     force_z_offset = msg.wrench.force.z
        #     force_sensor_flag = False
        # force_z = msg.wrench.force.z - force_z_offset











    # def haptic_force(x_p,y_p,z_p):

    #     global robot_ext_force, buffer_force_x, buffer_force_y, buffer_force_z, force_send_on_haptic 

    #     global pub_force

    #     omni_feedback_msg = OmniFeedback() 

    #     buffer_force_size = 100
    #     d = 0.145   ############# attachment length
            
        
    #     # # correct frames for device alignment
    #     # fx, fy, fz = fy, fx, -fz                                    #######  ????

    #     fx, fy, fz = robot_ext_force[0], robot_ext_force[1], robot_ext_force[2]
    #     F = [fx, fy, fz]    
        
    
    # #####################################   IF BUFFER IS CREATED AND AVERAGE IS TAKEN 

    #     buffer_force_x.append(fx)
    #     buffer_force_y.append(fy)
    #     buffer_force_z.append(fz)

    #     if len(buffer_force_x) > buffer_force_size:
    #         del buffer_force_x[0]
    #     if len(buffer_force_y) > buffer_force_size:
    #         del buffer_force_y[0]
    #     if len(buffer_force_z) > buffer_force_size:
    #         del buffer_force_z[0]

    #     buffer_force_x_avg = np.median(buffer_force_x)
    #     buffer_force_y_avg = np.median(buffer_force_y)
    #     buffer_force_z_avg = np.median(buffer_force_z)

        
    #     force_send_on_haptic = np.array([buffer_force_x_avg,buffer_force_y_avg,buffer_force_z_avg])

    #     force_condition = np.sqrt(buffer_force_x_avg**2 + buffer_force_y_avg**2 + buffer_force_z_avg**2)
        
    #     print("force condition: ",force_condition)
    #     if force_condition > 0.25:
            
    #         omni_feedback_msg.position.x = x_p
    #         omni_feedback_msg.position.y = y_p
    #         omni_feedback_msg.position.z = z_p
    #         omni_feedback_msg.force.x =   buffer_force_x_avg
    #         omni_feedback_msg.force.y =   buffer_force_y_avg
    #         omni_feedback_msg.force.z =   buffer_force_z_avg
    #     else:
    #         omni_feedback_msg.position.x = x_p
    #         omni_feedback_msg.position.y = y_p
    #         omni_feedback_msg.position.z = z_p
    #         omni_feedback_msg.force.x = 0
    #         omni_feedback_msg.force.y = 0
    #         omni_feedback_msg.force.z = 0



    #     print('Omni_feedback_msg: ',omni_feedback_msg)
    #     pub_force.publish(omni_feedback_msg)