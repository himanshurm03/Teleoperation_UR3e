#!/usr/bin/env python
import rospy
from teleop_msgs.msg import MasterToSlaveData
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from io import BytesIO

def measure_message_sizes():
    # Create a sample message
    msg = MasterToSlaveData()
    
    # Fill with sample data
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    
    msg.pose = Pose()
    msg.pose.position = Point(1.0, 2.0, 3.0)
    msg.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    
    msg.force_timestamp = 12345.67890
    
    # Serialize to get size
    buff = BytesIO()
    msg.serialize(buff)
    total_size = buff.tell()
    
    print("=" * 60)
    print("Message Component Sizes:")
    print("=" * 60)
    print(f"Total serialized message size: {total_size} bytes")
    print(f"\nBreakdown (approximate):")
    print(f"  - Header (with frame_id '{msg.header.frame_id}'): ~{12 + len(msg.header.frame_id)} bytes")
    print(f"  - Pose (position + orientation): 56 bytes")
    print(f"  - Force timestamp: 8 bytes")
    print("=" * 60)

if __name__ == '__main__':
    measure_message_sizes()