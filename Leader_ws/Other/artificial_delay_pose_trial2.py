#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from collections import deque
import numpy as np
import matplotlib.pyplot as plt

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

        # For plotting
        self.time_plot = []
        self.delay_plot = []
        self.delay_time_plot = []

    def update_list(self, time, delay, delay_time):
        self.time_plot.append(time)
        self.delay_plot.append(delay)
        self.delay_time_plot.append(delay_time)
        rospy.loginfo(f"updated") 

    def plot_data(self):
        time_plot = np.array(self.time_plot)
        delay_plot = np.array(self.delay_plot)
        delay_time_plot = np.array(self.delay_time_plot)

        plt.figure()
        plt.plot(time_plot, delay_plot, label="delay")
        plt.plot(time_plot, delay_time_plot, label="delay_time")
        plt.legend()  # Add the legend
        plt.show()

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
                self.update_list(current_time, current_time - timestamp, self.delay_time)
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
    finally:
        controller.plot_data()

'''second vala '''

# import rospy
# from geometry_msgs.msg import PoseStamped
# from collections import deque
# import random 
# import numpy as np
# import matplotlib.pyplot as plt

# class RobotEndEffectorController:

#     def __init__(self, min_delay_time, max_delay_time):
#         # Initialize ROS node
#         rospy.init_node('artificial_delay_pose_master', anonymous=True)

#         # Publisher
#         self.pub = rospy.Publisher("delayed_data_from_master", PoseStamped, queue_size=10)

#         # Subscriber
#         rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

#         # Time delay mechanism
#         self.min_delay_time = min_delay_time  # Minimum time delay in seconds
#         self.max_delay_time = max_delay_time  # Maximum time delay in seconds
#         self.delay_buffer = deque()  # Buffer will hold (timestamp, message, delay_time) tuples

#         # For plotting
#         self.time_plot = []
#         self.delay_plot = []
#         self.delay_time_plot = []

#     def update_list(self, time, delay, delay_time):
#         self.time_plot.append(time)
#         self.delay_plot.append(delay)
#         self.delay_time_plot.append(delay_time)
#         rospy.loginfo(f"updated") 

#     def plot_data(self):
#         time_plot = np.array(self.time_plot)
#         delay_plot = np.array(self.delay_plot)
#         delay_time_plot = np.array(self.delay_time_plot)

#         plt.figure()
#         plt.plot(time_plot, delay_plot, label="delay")
#         plt.plot(time_plot, delay_time_plot, label="delay_time")
#         plt.legend()  # Add the legend
#         plt.show()

#     def haptic_callback(self, msg: PoseStamped):
#         # Add received message to the buffer with a timestamp
#         current_time = rospy.get_time()
#         delay_time = random.uniform(self.min_delay_time, self.max_delay_time)
#         rospy.loginfo(f"Message received, will delay for: {delay_time:.2f} seconds")
#         self.delay_buffer.append((current_time, msg, delay_time))  # Include delay_time in the buffer

#     def delayed_data_callback(self, event):
#         # Check if there are any delayed messages ready to be published
#         current_time = rospy.get_time()

#         while len(self.delay_buffer) > 0:
#             timestamp, msg, delay_time = self.delay_buffer[0]  # Get the oldest message and its delay time

#             # Check if enough time has passed for this message
#             if current_time - timestamp >= delay_time:
#                 self.pub.publish(msg)  # Publish the delayed message
#                 rospy.loginfo(f"Published message at {current_time} after delay of {delay_time:.2f} seconds")
#                 self.delay_buffer.popleft()  # Remove it from the buffer
#                 self.update_list(current_time, current_time - timestamp, delay_time)
#             else:
#                 break  # The message hasn't been delayed long enough, stop checking


#     def main_loop(self):
#         # Set a timer to call the delayed_data_callback at 1000 Hz (matches incoming message rate)
#         rate = 1000  # Timer update rate
#         rospy.Timer(rospy.Duration(1.0 / rate), self.delayed_data_callback)
#         rospy.spin()

# if __name__ == "__main__":
#     try:
#         min_delay_time = 0.01  # Minimum delay time (1 second)
#         max_delay_time = 0.2 # Maximum delay time (5 seconds)
#         controller = RobotEndEffectorController(min_delay_time, max_delay_time)
#         controller.main_loop()  # Ensure this method exists
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         controller.plot_data()

'''first ka second trial'''

# import rospy
# from geometry_msgs.msg import PoseStamped
# from collections import deque
# import numpy as np
# import matplotlib.pyplot as plt

# class RobotEndEffectorController:

#     def __init__(self, delay_time):
#         # Initialize ROS node
#         rospy.init_node('artificial_delay_pose_master', anonymous=True)

#         # Publisher
#         self.pub = rospy.Publisher("delayed_data_from_master", PoseStamped, queue_size=10)

#         # Subscriber
#         rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

#         # Time delay mechanism
#         self.delay_time = delay_time  # Time delay in seconds
#         self.delay_buffer = deque()  # Buffer will hold (timestamp, message) pairs

#         # For plotting
#         self.time_plot = []
#         self.delay_plot = []
#         self.delay_time_plot = []

#     def update_list(self, time, delay, delay_time):
#         self.time_plot.append(time)
#         self.delay_plot.append(delay)
#         self.delay_time_plot.append(delay_time)
#         rospy.loginfo(f"updated") 

#     def plot_data(self):
#         time_plot = np.array(self.time_plot)
#         delay_plot = np.array(self.delay_plot)
#         delay_time_plot = np.array(self.delay_time_plot)

#         plt.figure()
#         plt.plot(time_plot, delay_plot, label="delay")
#         plt.plot(time_plot, delay_time_plot, label="delay_time")
#         plt.legend()  # Add the legend
#         plt.show()

#     def haptic_callback(self, msg: PoseStamped):
#         # Add received message to the buffer with a timestamp
#         current_time = rospy.get_time()
#         rospy.loginfo(f"Message received, buffer length: {len(self.delay_buffer)}") 
#         self.delay_buffer.append((current_time, msg))

#     def delayed_data_callback(self, event):
#         # Check if there are any delayed messages ready to be published
#         current_time = rospy.get_time()

#         while len(self.delay_buffer) > 0:
#             flag = True
#             timestamp, msg = self.delay_buffer[0]  # Peek at the oldest message

#             while flag:
#                 if timestamp < current_time  - self.delay_time:
#                     self.delay_buffer.popleft()  # Remove it from the buffer
#                     timestamp, msg = self.delay_buffer[0]
#                 else:
#                     self.pub.publish(msg) 
#                     self.update_list(current_time, current_time - timestamp, self.delay_time)
#                     flag = False

#     def main_loop(self):
#         # Set a timer to call the delayed_data_callback at 1000 Hz (matches incoming message rate)
#         rate = 1000  # Timer update rate
#         rospy.Timer(rospy.Duration(1.0 / rate), self.delayed_data_callback)
#         rospy.spin()

# if __name__ == "__main__":
#     try:
#         delay_time = 1.0  # 1 second delay
#         controller = RobotEndEffectorController(delay_time)
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         controller.plot_data()

'''first ka third trial'''

# import rospy
# from geometry_msgs.msg import PoseStamped
# import matplotlib.pyplot as plt

# class DelayPoseMaster:
#     def __init__(self, delay_time):
#         # Initialize ROS node
#         rospy.init_node('artificial_delay_pose_master', anonymous=True)

#         # Publisher
#         self.pub = rospy.Publisher("delayed_data_from_master", PoseStamped, queue_size=10)

#         # Subscriber
#         rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

#         # Store the delay time (dt)
#         self.delay_time = delay_time

#         # Lists to store timestamps
#         self.received_times = []
#         self.published_times = []

#     def haptic_callback(self, msg):
#         # Get the current time t when the message is received
#         receive_time = rospy.get_time()
#         rospy.loginfo(f"Message received at time: {receive_time}")

#         # Store the received time
#         self.received_times.append(receive_time)

#         # Use a Timer to delay publishing the message by dt seconds
#         rospy.Timer(rospy.Duration(self.delay_time), lambda event: self.publish_delayed_msg(msg), oneshot=True)

#     def publish_delayed_msg(self, msg):
#         # Get the current time when the message is being published
#         publish_time = rospy.get_time()
#         rospy.loginfo(f"Publishing delayed message at time: {publish_time}")

#         # Store the published time only if there's a corresponding received time
#         if len(self.published_times) < len(self.received_times):
#             self.published_times.append(publish_time)

#         # Publish the delayed message
#         self.pub.publish(msg)

#     def plot_delay(self):
#         # Ensure that we only use the pairs that have both received and published times
#         if len(self.published_times) < len(self.received_times):
#             self.received_times = self.received_times[:len(self.published_times)]
        
#         # Calculate the delays
#         delays = [pub_time - recv_time for recv_time, pub_time in zip(self.received_times, self.published_times)]

#         # Plot the delays
#         plt.figure(figsize=(10, 6))
#         plt.plot(self.received_times, delays, marker='o', linestyle='-', color='b')
#         plt.title("Message Delay vs. Time of Reception")
#         plt.xlabel("Message Reception Time (s)")
#         plt.ylabel("Delay (s)")
#         plt.grid(True)
#         plt.show()

# if __name__ == '__main__':
#     try:
#         # Set the desired delay time (dt)
#         delay_time = 0.2  # For example, 2 seconds

#         # Create the DelayPoseMaster object
#         dpm = DelayPoseMaster(delay_time)

#         # Keep the node running
#         rospy.spin()

#         # After shutting down, plot the delay
#         dpm.plot_delay()

#     except rospy.ROSInterruptException:
#         pass

'''first ka third trial bina plot ke'''

# import rospy
# from geometry_msgs.msg import PoseStamped

# class DelayPoseMaster:
#     def __init__(self, delay_time):
#         # Initialize ROS node
#         rospy.init_node('artificial_delay_pose_master', anonymous=True)

#         # Publisher
#         self.pub = rospy.Publisher("delayed_data_from_master", PoseStamped, queue_size=10)

#         # Subscriber
#         rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

#         # Store the delay time (dt)
#         self.delay_time = delay_time

#     def haptic_callback(self, msg):
#         # Get the current time t when the message is received
#         rospy.loginfo(f"Message received at time: {rospy.get_time()}")

#         # Use a Timer to delay publishing the message by dt seconds
#         rospy.Timer(rospy.Duration(self.delay_time), lambda event: self.publish_delayed_msg(msg), oneshot=True)

#     def publish_delayed_msg(self, msg):
#         # Publish the delayed message
#         rospy.loginfo(f"Publishing delayed message at time: {rospy.get_time()}")
#         self.pub.publish(msg)

# if __name__ == '__main__':
#     try:
#         # Set the desired delay time (dt)
#         delay_time = 2.0  # For example, 2 seconds

#         # Create the DelayPoseMaster object
#         dpm = DelayPoseMaster(delay_time)

#         # Keep the node running
#         rospy.spin()

#     except rospy.ROSInterruptException:
#         pass
