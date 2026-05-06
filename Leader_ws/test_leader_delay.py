#!/usr/bin/env python3

#Dissimilarity Rating :Final
#Delay



import rospy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import tkinter as tk
import random
import subprocess
import csv
import os
from datetime import datetime

import cv2

INTERFACE = "enp2s0"
TOTAL_DELAY = 16


class Experiment:

    def __init__(self):

        rospy.init_node('master_rating_experiment')
        self.pub = rospy.Publisher('/delay_cmd', Int32MultiArray, queue_size=10)
        self.bridge = CvBridge()
        rospy.Subscriber('/camera_feed', Image, self.image_callback)
        self.root = tk.Tk()
        self.root.title("Delay Rating Experiment")
        self.root.geometry("800x400+1120+0")

        self.stage = "name"
        self.trial = 0

        
        self.asym_pairs = [
            (1,15), (15,1),
            (3,13), (13,3),
            (5,11), (11,5)
        ]
        random.shuffle(self.asym_pairs)

        self.order_toggle = True

        self.label = tk.Label(self.root, text="Enter Name and Press ENTER",
                              font=("Arial", 20))
        self.label.pack()

        self.entry = tk.Entry(self.root, font=("Arial", 16))
        self.entry.pack()
        self.entry.focus()

        self.root.bind("<Return>", self.handle_enter)
        self.root.bind("<Key>", self.key_handler)

    def image_callback(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_image = cv2.resize(cv_image, (1000, 700))

        cv2.imshow("Camera Feed", cv_image)
        cv2.resizeWindow("Camera Feed", 1100, 700)
        cv2.moveWindow("Camera Feed", 0, 0)
        cv2.waitKey(1)

    def handle_enter(self, event):

        if self.stage == "name":

            self.username = self.entry.get().strip()
            if not self.username:
                return

            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

            base_path = "/home/autonomous-lab/Desktop/delay/DS"
            os.makedirs(base_path, exist_ok=True)

            self.filename = os.path.join(base_path,
                                        f"{self.username}_{timestamp}.csv")

            self.entry.pack_forget()
            self.stage = "ready"
            self.label.config(text="Press S to Start Trial")


    def key_handler(self, event):

        if self.stage == "ready" and event.char.lower() == 's':
            self.start_trial()

        elif self.stage == "phase":

            if event.keysym == 'space':
                self.next_phase()

            elif event.char.lower() == 'r' and self.current_phase == 2:
                self.repeat_trial()

        
        elif self.stage == "response":

            if event.char.isdigit():
                rating = int(event.char)
 
                if 0 <= rating <= 9:
                    self.user_rating = rating
                    self.save_trial()
                    self.stage = "ready"
                    self.label.config(text="Saved.\nPress S for next trial")

            elif event.char.lower() == 't':
                self.user_rating = 10
                self.save_trial()
                self.stage = "ready"
                self.label.config(text="Saved.\nPress S for next trial")


    def apply_delay(self, m, s):

        subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
                        shell=True)

        subprocess.call(
            f"sudo tc qdisc add dev {INTERFACE} root netem delay {m}ms",
            shell=True)

        msg = Int32MultiArray()
        msg.data = [m, s]
        self.pub.publish(msg)


    def start_trial(self):

        if len(self.asym_pairs) == 0:
            self.label.config(text="All trials completed!")
            return

        self.trial += 1

        sym = (8, 8)
        self.asym = self.asym_pairs.pop()

        if self.order_toggle:
            self.phases = [
                {"type": "sym", "delay": sym},
                {"type": "asym", "delay": self.asym}
            ]
        else:
            self.phases = [
                {"type": "asym", "delay": self.asym},
                {"type": "sym", "delay": sym}
            ]

        self.order_toggle = not self.order_toggle

        self.current_phase = 0
    
        self.run_phase()

  
    def run_phase(self):

        if self.current_phase >= 2:
            self.ask_response()
            return

        self.stage = "phase"

        phase = self.phases[self.current_phase]
        m, s = phase["delay"]

        self.apply_delay(m, s)

        if self.current_phase == 0:
            extra = "\nPress SPACE to continue"
        else:
            extra = "\nPress SPACE to continue | Press R to repeat BOTH phases"

        self.label.config(
            text=f"Phase {self.current_phase + 1}\nExplore...{extra}"
        )

        self.current_phase += 1


    def repeat_trial(self):

        self.current_phase = 0
        self.label.config(text="Repeating both phases...\nStarting Phase 1")
        self.run_phase()


    def next_phase(self):
        self.run_phase()

    def ask_response(self):

        self.stage = "response"

        self.label.config(
            text="Rate Dissimilarity (0–10)\n\n"
                 "0 = Same\n"
                 "10 = Completely different\n\n"
                 "Press 0–9 or T for 10"
                 
        )


    def save_trial(self):

        file_exists = os.path.isfile(self.filename)

        with open(self.filename, mode='a', newline='') as file:
            writer = csv.writer(file)

            if not file_exists:
                writer.writerow([
                    "Timestamp", "User", "Trial",
                    "Phase1", "Phase2",
                    "AsymPair",
                    "Rating"
                ])

            writer.writerow([
                datetime.now(),
                self.username,
                self.trial,
                self.phases[0]["delay"],
                self.phases[1]["delay"],
                self.asym,
                self.user_rating
            ])

  
    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    Experiment().run()

