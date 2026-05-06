#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from collections import deque
import random  # Import the random module

class RobotEndEffectorController:

    def __init__(self, min_delay_time, max_delay_time):
        # Initialize ROS node
        rospy.init_node('artificial_delay_pose_master', anonymous=True)

        # Publisher
        self.pub = rospy.Publisher("delayed_data_from_master", PoseStamped, queue_size=10)

        # Subscriber
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

        # Time delay mechanism
        self.min_delay_time = min_delay_time  # Minimum time delay in seconds
        self.max_delay_time = max_delay_time  # Maximum time delay in seconds
        self.delay_buffer = deque()  # Buffer will hold (timestamp, message, delay_time) tuples

    def haptic_callback(self, msg: PoseStamped):
        # Add received message to the buffer with a timestamp
        current_time = rospy.get_time()
        delay_time = random.uniform(self.min_delay_time, self.max_delay_time)
        rospy.loginfo(f"Message received, will delay for: {delay_time:.2f} seconds")
        self.delay_buffer.append((current_time, msg, delay_time))  # Include delay_time in the buffer

    def delayed_data_callback(self, event):
        # Check if there are any delayed messages ready to be published
        current_time = rospy.get_time()

        while len(self.delay_buffer) > 0:
            timestamp, msg, delay_time = self.delay_buffer[0]  # Get the oldest message and its delay time

            # Check if enough time has passed for this message
            if current_time - timestamp >= delay_time:
                self.pub.publish(msg)  # Publish the delayed message
                rospy.loginfo(f"Published message at {current_time} after delay of {delay_time:.2f} seconds")
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
        min_delay_time = 0.01  # Minimum delay time (1 second)
        max_delay_time = 0.2 # Maximum delay time (5 seconds)
        controller = RobotEndEffectorController(min_delay_time, max_delay_time)
        controller.main_loop()  # Ensure this method exists
    except rospy.ROSInterruptException:
        pass
