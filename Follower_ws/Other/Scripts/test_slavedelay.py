#!/usr/bin/env python3

#################################################################
#This is replicating the delay perfectly just need to change the interface


# import rospy
# from std_msgs.msg import Int32
# import subprocess

# INTERFACE = "enx7cc2c648fa5e"

# def apply_delay(msg):

#     delay = msg.data

#     subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root",
#                     shell=True)

#     subprocess.call(
#         f"sudo tc qdisc add dev {INTERFACE} root netem delay {delay}ms",
#         shell=True)

#     rospy.loginfo(f"Applied symmetric delay: {delay} ms")

# def main():
#     rospy.init_node('slave_delay_node')
#     rospy.Subscriber('/delay_cmd', Int32, apply_delay)
#     rospy.spin()

# if __name__ == "__main__":
#     main()


####################################################################################
#Symmetrical Delays

# import rospy
# from std_msgs.msg import Int32
# import subprocess

# INTERFACE = "enx7cc2c648fa5e"

# def apply_delay(msg):

#     delay = msg.data

#     subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root",
#                     shell=True)

#     subprocess.call(
#         f"sudo tc qdisc add dev {INTERFACE} root netem delay {delay}ms",
#         shell=True)

#     rospy.loginfo(f"Applied delay: {delay} ms")

# def main():
#     rospy.init_node('slave_delay_node')
#     rospy.Subscriber('/delay_cmd', Int32, apply_delay)
#     rospy.spin()

# if __name__ == "__main__":
#     main()




####################################################################################################

#Asymmetrical Delays



# import rospy
# from std_msgs.msg import Int32MultiArray
# import subprocess
# import atexit

# INTERFACE = "enx7cc2c648fa5e"   # Change if needed


# def apply_delay(msg):
#     """
#     msg.data = [master_delay, slave_delay]
#     Slave applies only slave_delay
#     """

#     if len(msg.data) < 2:
#         return

#     slave_delay = msg.data[1]

#     # Clear previous delay
#     subprocess.call(
#         f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
#         shell=True
#     )

#     # Apply new delay
#     subprocess.call(
#         f"sudo tc qdisc add dev {INTERFACE} root netem delay {slave_delay}ms",
#         shell=True
#     )


# def cleanup():
#     """
#     Ensures delay is removed if node exits or crashes
#     """
#     subprocess.call(
#         f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
#         shell=True
#     )


# def main():

#     # Disable INFO prints completely
#     rospy.init_node('slave_delay_node', log_level=rospy.ERROR)

#     rospy.Subscriber('/delay_cmd', Int32MultiArray, apply_delay)

#     # Auto cleanup on exit
#     atexit.register(cleanup)

#     rospy.spin()


# if __name__ == "__main__":
#     main()


#####################################################################################################

#Delay:Final


# import rospy
# from std_msgs.msg import Int32MultiArray
# import subprocess
# import atexit

# INTERFACE = "enx7cc2c648fa5e"

# def apply_delay(msg):

#     if len(msg.data) < 2:
#         return

#     slave_delay = msg.data[1]

#     subprocess.call(
#         f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
#         shell=True
#     )

#     subprocess.call(
#         f"sudo tc qdisc add dev {INTERFACE} root netem delay {slave_delay}ms",
#         shell=True
#     )

# def cleanup():
#     subprocess.call(
#         f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
#         shell=True
#     )

# def main():
#     rospy.init_node('slave_delay_node', log_level=rospy.ERROR)
#     rospy.Subscriber('/delay_cmd', Int32MultiArray, apply_delay)
#     atexit.register(cleanup)
#     rospy.spin()

# if __name__ == "__main__":
#     main()


########################################################################################3

#Packet loss


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

    # Clear previous qdisc
    subprocess.call(
        f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
        shell=True
    )

    # Apply packet loss
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

    # 🔥 Updated topic
    rospy.Subscriber('/loss_cmd', Int32MultiArray, apply_loss)

    atexit.register(cleanup)
    rospy.spin()

if __name__ == "__main__":
    main()