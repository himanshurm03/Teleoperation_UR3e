#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import subprocess
import atexit

INTERFACE = "enx7cc2c648fa5e"

def apply_loss(msg):

    if len(msg.data) < 2:
        return

    slave_loss = msg.data[1]


    subprocess.call(
        f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
        shell=True
    )

   
    subprocess.call(
        f"sudo tc qdisc add dev {INTERFACE} root netem loss {slave_loss}%",
        shell=True
    )

def cleanup():
    subprocess.call(
        f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
        shell=True
    )

def main():
    rospy.init_node('slave_loss_node', log_level=rospy.ERROR)

    
    rospy.Subscriber('/loss_cmd', Int32MultiArray, apply_loss)

    atexit.register(cleanup)
    rospy.spin()

if __name__ == "__main__":
    main()