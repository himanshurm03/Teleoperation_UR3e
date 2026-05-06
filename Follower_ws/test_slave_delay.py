#!/usr/bin/env python3


import rospy
from std_msgs.msg import Int32MultiArray
import subprocess
import atexit

INTERFACE = "enx7cc2c648fa5e"

def apply_delay(msg):

    if len(msg.data) < 2:
        return

    slave_delay = msg.data[1]

    subprocess.call(
        f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
        shell=True
    )

    subprocess.call(
        f"sudo tc qdisc add dev {INTERFACE} root netem delay {slave_delay}ms",
        shell=True
    )

def cleanup():
    subprocess.call(
        f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
        shell=True
    )

def main():
    rospy.init_node('slave_delay_node', log_level=rospy.ERROR)
    rospy.Subscriber('/delay_cmd', Int32MultiArray, apply_delay)
    atexit.register(cleanup)
    rospy.spin()

if __name__ == "__main__":
    main()


