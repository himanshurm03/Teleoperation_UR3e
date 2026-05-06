#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from collections import deque

# # class RobotEndEffectorController:

# #     def __init__(self, delay_time):
# #         # Initialize ROS node
# #         rospy.init_node('artificial_delay_pose_master', anonymous=True)

# #         # Publisher
# #         self.pub = rospy.Publisher("delayed_data_from_master", PoseStamped, queue_size=10)

# #         # Subscriber
# #         rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

# #         # Time delay mechanism
# #         rate = 1000  # Update rate in Hz
# #         self.delay_time = delay_time  # Time delay in seconds
# #         self.delay_buffer = deque(maxlen=int(delay_time * rate))  # Buffer based on delay time and update rate

# #     def haptic_callback(self, msg: PoseStamped):
# #         # Add received message to the buffer
# #         rospy.loginfo("Message received in callback")
# #         self.delay_buffer.append(msg)

# #     def delayed_data_callback(self, event):
# #         # Check if there are any delayed messages ready to be published
# #         if len(self.delay_buffer) > 0:
# #             msg = self.delay_buffer.popleft()  # Get the delayed message
# #             self.pub.publish(msg)  # Publish the delayed message
# #         else:
# #             rospy.logwarn("Delay buffer is empty, not publishing.")

# #     def main_loop(self):
# #         rate = 1000  # Timer update rate (1000 Hz)
# #         # Setup a timer to call the delayed_data_callback at the specified rate
# #         rospy.Timer(rospy.Duration(1.0 / rate), self.delayed_data_callback)
# #         rospy.spin()

# # if __name__ == "__main__":
# #     try:
# #         delay_time = 1.0  # 1 second delay
# #         controller = RobotEndEffectorController(delay_time)
# #         controller.main_loop()
# #     except rospy.ROSInterruptException:
# #         pass

# class RobotEndEffectorController:

#     def __init__(self, delay_time):
#         # Initialize ROS node
#         rospy.init_node('artificial_delay_pose_master', anonymous=True)

#         # Publisher
#         self.pub = rospy.Publisher("delayed_data_from_master", PoseStamped, queue_size=10)

#         # Subscriber
#         rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

#         # Time delay mechanism
#         rate = 1000  # Incoming message rate in Hz (1000 Hz)
#         self.delay_time = delay_time  # Time delay in seconds
#         self.delay_buffer = deque(maxlen=int(delay_time * rate))  # Buffer size (1000 messages for 1 second)

#         self.start_time = rospy.get_time()

#     def haptic_callback(self, msg: PoseStamped):
#         # Add received message to the buffer
#         #rospy.loginfo(f"Message received, buffer length: {len(self.delay_buffer)}")  # Log buffer length
#         self.delay_buffer.append(msg)

#     def delayed_data_callback(self, event):
#         # Publish only if the buffer is full (i.e., has delayed messages)
#         if rospy.get_time() - self.start_time <= self.delay_time:
#             return
#         if len(self.delay_buffer) == self.delay_buffer.maxlen:
#             msg = self.delay_buffer.popleft()  # Get the oldest message (delayed)
#             #rospy.loginfo(f"Publishing delayed message, buffer length: {len(self.delay_buffer)}")
#             self.pub.publish(msg)  # Publish the delayed message
#         else:
#             rospy.logwarn(f"Buffer not full yet (length: {len(self.delay_buffer)}), not publishing.")

#     def main_loop(self):
#         # Set a timer to call the delayed_data_callback at 1 Hz (once per second)
#         rate = 1000
#         rospy.Timer(rospy.Duration(1.0/rate), self.delayed_data_callback)
#         rospy.spin()

# if __name__ == "__main__":
#     try:
#         delay_time = 1.0  # 1 second delay
#         controller = RobotEndEffectorController(delay_time)
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass

class RobotEndEffectorController:

    def __init__(self, delay_time):
        # Initialize ROS node
        rospy.init_node('artificial_delay_pose_master', anonymous=True)

        # Publisher
        self.pub = rospy.Publisher("delayed_data_from_master", PoseStamped, queue_size=10)

        # Subscriber
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

        # Time delay mechanism
        self.delay_time = delay_time  # Time delay in seconds
        self.delay_buffer = deque()  # Buffer will hold (timestamp, message) pairs

    def haptic_callback(self, msg: PoseStamped):
        # Add received message to the buffer with a timestamp
        current_time = rospy.get_time()
        rospy.loginfo(f"Message received, buffer length: {len(self.delay_buffer)}") 
        self.delay_buffer.append((current_time, msg))

    def delayed_data_callback(self, event):
        # Check if there are any delayed messages ready to be published
        current_time = rospy.get_time()

        while len(self.delay_buffer) > 0:
            timestamp, msg = self.delay_buffer[0]  # Peek at the oldest message

            # Check if the message has been in the buffer long enough (delay_time)
            if current_time - timestamp >= self.delay_time:
                self.pub.publish(msg)  # Publish the delayed message
                self.delay_buffer.popleft()  # Remove it from the buffer
            else:
                break  # The message hasn't been delayed long enough, stop checking

    def main_loop(self):
        # Set a timer to call the delayed_data_callback at 1000 Hz (matches incoming message rate)
        rate = 1000  # Timer update rate
        rospy.Timer(rospy.Duration(1.0 / rate), self.delayed_data_callback)
        rospy.spin()

if __name__ == "__main__":
    try:
        delay_time = 1.0  # 1 second delay
        controller = RobotEndEffectorController(delay_time)
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
