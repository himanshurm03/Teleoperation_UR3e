#!/usr/bin/env python3
# import rospy
# import time
# from omni_msgs.msg import OmniFeedback 

# class HapticForceController:
#     def __init__(self):
#         rospy.init_node('master_force', anonymous=True)
#         self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)
#         rospy.Subscriber('force_from_slave_to_master', OmniFeedback, self.force_from_slave_to_master_callback)
#         rospy.on_shutdown(self.shutdown_hook)
#         self.shutdown_flag = False

#     def force_from_slave_to_master_callback(self, msg: OmniFeedback):
#         if self.shutdown_flag: 
#             return
#         self.force_pub.publish(msg)

#     def main_loop(self):
#         rospy.spin()

#     def shutdown_hook(self):
#         self.shutdown_flag = True
#         zero_force_msg = OmniFeedback()
#         zero_force_msg.force.x = 0
#         zero_force_msg.force.y = 0
#         zero_force_msg.force.z = 0
#         rospy.loginfo("Sending zero force to haptic device...")
#         for _ in range(10): 
#             self.force_pub.publish(zero_force_msg)
#             time.sleep(0.1)
#         rospy.loginfo("Shutting down completed.")

# if __name__ == "__main__":
#     try:
#         controller = HapticForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass

'''delay computation'''
import rospy
import time
from omni_msgs.msg import OmniFeedback
from std_msgs.msg import Float64 

class HapticForceController:
    def __init__(self):
        rospy.init_node('master_force', anonymous=True)

        # Publishers
        self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)
        self.time_pub = rospy.Publisher('time_from_master_to_slave', Float64, queue_size=1)

        # Subscribers
        rospy.Subscriber('force_from_slave_to_master', OmniFeedback, self.force_from_slave_to_master_callback)
        rospy.Subscriber('time_from_slave_to_master', Float64, self.time_from_slave_to_master_callback)
        
        # Shutdown behavior
        rospy.on_shutdown(self.shutdown_hook)

        self.shutdown_flag = False

    def time_from_slave_to_master_callback(self, msg: Float64):
        # Compute the delay using the message data
        delay = (rospy.get_time() - msg.data) / 2
        rospy.loginfo(f"Delay for force: {delay}")

    def force_from_slave_to_master_callback(self, msg: OmniFeedback):
        if self.shutdown_flag: 
            return
        # Publish the force feedback message
        self.force_pub.publish(msg)
        
        # Publish the current time to the time topic
        time_msg = Float64()
        time_msg.data = rospy.get_time()
        self.time_pub.publish(time_msg)

    def main_loop(self):
        rospy.spin()

    def shutdown_hook(self):
        # Ensure zero force is sent on shutdown
        self.shutdown_flag = True
        zero_force_msg = OmniFeedback()
        zero_force_msg.force.x = 0
        zero_force_msg.force.y = 0
        zero_force_msg.force.z = 0
        rospy.loginfo("Sending zero force to haptic device...")
        for _ in range(10): 
            self.force_pub.publish(zero_force_msg)
            time.sleep(0.1)
        rospy.loginfo("Shutting down completed.")

if __name__ == "__main__":
    try:
        controller = HapticForceController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
