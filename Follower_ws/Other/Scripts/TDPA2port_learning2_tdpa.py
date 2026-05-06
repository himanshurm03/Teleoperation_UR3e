#!/usr/bin/env python3

'''trial1'''
# import rospy
# import numpy as np
# from std_msgs.msg import Float64MultiArray
# import time

# class TDPAslave:

#     def __init__(self):

#         # Initialize node
#         rospy.init_node('Slave_tdpa', anonymous=True)
#         self.E_M_in = 0.0
#         self.E_S_out = 0.0
#         self.E_S_in = 0.0
#         self.vsd = np.zeros(3)
#         self.fs = np.zeros(3)
#         self.prev_time = None

#         # Publisher
#         self.velocity_publisher = rospy.Publisher("corrected_haptic_velocity", Float64MultiArray, queue_size=1)
#         self.energy_publisher = rospy.Publisher("energy_from_slave_to_master", Float64MultiArray, queue_size=1)

#         # Subscribers
#         rospy.Subscriber("message_from_master_to_slave", Float64MultiArray, self.velocity_and_energy_callback)
#         rospy.Subscriber('corrected_force', Float64MultiArray, self.robot_force_callback)

#     @staticmethod
#     def E_S_calculation(fs, vsd, E_S_out, E_S_in, delta_t):
#         P2 = fs[2]*vsd[2]
#         if P2<0:
#             E_S_out -= delta_t*P2
#         elif P2>0:
#             E_S_in += delta_t*P2
#         return E_S_out, E_S_in

#     @staticmethod
#     def beta_calculation(E_S_out, E_M_in, fs, delta_t):
#         if delta_t <= 0 or abs(fs[2]) < 0.001:
#             return 0.0 
#         if E_S_out > E_M_in:
#             #print(f"E_S_out: {E_S_out}, E_M_in: {E_M_in}, fs: {fs[2]}, delta_t: {delta_t}")
#             return (E_S_out - E_M_in)/(delta_t * (fs[2]**2))
#         else:
#             return 0.0
        
#     def robot_force_callback(self, msg: Float64MultiArray):
#         self.fs = np.array(msg.data)

#     def velocity_and_energy_callback(self, msg: Float64MultiArray):

#         self.E_M_in = np.array(msg.data[:1]).item() 
#         self.vsd = np.array(msg.data[1:]) 
#         #print(f"E_M_in: {self.E_M_in}, vsd: {self.vsd}")

#         current_time = time.time()
#         if self.prev_time is not None:
#             delta_t = current_time-self.prev_time
#             self.E_S_out, self.E_S_in = self.E_S_calculation(self.fs, self.vsd, self.E_S_out, self.E_S_in, delta_t)
#             beta = self.beta_calculation(self.E_S_out, self.E_M_in, self.fs, delta_t)
#         else:
#             beta = 0.0
#         self.prev_time = current_time

#         corrected_haptic_velocity = self.vsd + beta*self.fs
#         print(f"beta: {beta}, corrected_velocity: {corrected_haptic_velocity[2]}")

#         velocity_pub_msg = Float64MultiArray()
#         velocity_pub_msg.data = corrected_haptic_velocity.tolist()
#         self.velocity_publisher.publish(velocity_pub_msg)

#         energy_pub_msg = Float64MultiArray()
#         energy_pub_msg.data = [self.E_S_in]
#         self.energy_publisher.publish(energy_pub_msg)

#     def main_loop(self):
#         rospy.spin()

# if __name__ == "__main__":
#     try:
#         node = TDPAslave()
#         node.main_loop()
#     except rospy.ROSInterruptException:
#         pass




'''trial2'''
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
import time

class TDPAslave:

    def __init__(self):

        # Initialize node
        rospy.init_node('Slave_tdpa', anonymous=True)
        self.E_M_in = 0.0
        self.E_S_out = 0.0
        self.E_S_in = 0.0
        self.vsd = np.zeros(3)
        self.fs = np.zeros(3)
        self.prev_time = None

        # Publisher
        self.velocity_publisher = rospy.Publisher("corrected_haptic_velocity", Float64MultiArray, queue_size=1)
        self.energy_publisher = rospy.Publisher("energy_from_slave_to_master", Float64MultiArray, queue_size=1)

        # Subscribers
        rospy.Subscriber("message_from_master_to_slave", Float64MultiArray, self.velocity_and_energy_callback)
        rospy.Subscriber('corrected_force', Float64MultiArray, self.robot_force_callback)

    @staticmethod
    def E_S_calculation(fs, vsd, E_S_out, E_S_in, delta_t):
        P2 = fs[2]*vsd[2]
        if P2<0:
            E_S_out -= delta_t*P2
        elif P2>0:
            E_S_in += delta_t*P2
        return E_S_out, E_S_in

    @staticmethod
    def beta_calculation(E_S_out, E_M_in, fs, delta_t):
        if delta_t <= 0 or abs(fs[2]) < 0.1:
            return 0.0 
        if E_S_out > E_M_in:
            # print(f"E_S_out: {E_S_out}, E_M_in: {E_M_in}, fs: {fs[2]}, delta_t: {delta_t}")
            return (E_S_out - E_M_in)/(delta_t * (fs[2]**2))
        else:
            return 0.0
        
    def robot_force_callback(self, msg: Float64MultiArray):
        self.fs = np.array(msg.data)

    def velocity_and_energy_callback(self, msg: Float64MultiArray):

        self.E_M_in = np.array(msg.data[:1]).item() 
        self.vsd = np.array(msg.data[1:]) 
        #print(f"E_M_in: {self.E_M_in}, vsd: {self.vsd}")

        current_time = time.time()
        if self.prev_time is not None:
            delta_t = current_time-self.prev_time
            self.E_S_out, self.E_S_in = self.E_S_calculation(self.fs, self.vsd, self.E_S_out, self.E_S_in, delta_t)
            beta = self.beta_calculation(self.E_S_out, self.E_M_in, self.fs, delta_t)
        else:
            beta = 0.0
        self.prev_time = current_time

        #corrected_haptic_velocity = self.vsd + beta*self.fs
        corrected_haptic_velocity = self.vsd 
        print(f"beta: {beta}, corrected_velocity: {corrected_haptic_velocity[2]}, abs_force: {abs(self.fs[2])}")

        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = corrected_haptic_velocity.tolist()
        self.velocity_publisher.publish(velocity_pub_msg)

        energy_pub_msg = Float64MultiArray()
        energy_pub_msg.data = [self.E_S_in]
        self.energy_publisher.publish(energy_pub_msg)

    def main_loop(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = TDPAslave()
        node.main_loop()
    except rospy.ROSInterruptException:
        pass