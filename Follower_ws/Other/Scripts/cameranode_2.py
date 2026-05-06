#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import sys
from rospy import Time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

delay = [0.0, 0.0, 0.0]

def capture_and_publish():
    rospy.init_node('camera_client_node', anonymous=True)
    camera_topic = '/usb_cam/image_raw'
    image_pub = rospy.Publisher('/camera_feed', Image, queue_size= None)
    pubdelay = rospy.Publisher("cam_delay", Float32MultiArray, queue_size=10)
    pubcamcumdelay = rospy.Publisher("cam_cum_out", String, queue_size=10)
    
    # Set camera resolution to 640x480
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 752) # 320
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 416) # 176
    
    # You can try setting the frame rate (FPS) if your camera supports it
    # Uncomment the following line and adjust the desired FPS value
    #cap.set(cv2.CAP_PROP_FPS, 30)  # Adjust the desired FPS
    cap.set(cv2.CAP_PROP_FPS, 30)  # Adjust the desired FPS
    
    bridge = CvBridge()

    rate = rospy.Rate(30)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        global delay
        cam_delay = Float32MultiArray()

        t_ime_1 = rospy.get_time()
        ret, frame = cap.read()
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        t_ime_2 = rospy.get_time()
        delaycapture = t_ime_2 - t_ime_1
        # print("delaycapture",delaycapture)
        if ret:
            t_ime_3 = rospy.get_time()
            image_message = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            t_ime_4 = rospy.get_time()
            delayconvert = t_ime_4 - t_ime_3

            delay[0] = delaycapture
            delay[1] = delayconvert
            cam_delay.data = delay

            ##print('capd',delaycapture,'convd',delayconvert)####wireshark
            camcumdelay = "%s" % (rospy.get_time() - 1702595722.051502)
            # rospy.loginfo(camcumdelay)
            image_size = sys.getsizeof(image_message.data)
            ###########rospy.loginfo(f"size of the image pcket: {image_size} bytes")
            image_pub.publish(image_message)
            #####pubdelay.publish(cam_delay)  ###wireshark
            ####pubcamcumdelay.publish(camcumdelay)  ##wireshark



        rate.sleep()

if __name__ == '__main__':
    try:
        capture_and_publish()
    except rospy.ROSInterruptException:
        pass
