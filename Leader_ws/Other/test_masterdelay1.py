#!/usr/bin/env python3

#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import tkinter as tk
import random
import subprocess
import cv2

INTERFACE = "enp2s0"


TRAINING_DELAY = (1, 25)


class TrainingExperiment:

    def __init__(self):

        rospy.init_node('delay_training_node')
        self.pub = rospy.Publisher('/delay_cmd', Int32MultiArray, queue_size=10)

        # Camera
        self.bridge = CvBridge()
        rospy.Subscriber('/camera_feed', Image, self.image_callback)

        # GUI
        self.root = tk.Tk()
        self.root.title("Delay Training")
        self.root.geometry("800x400+1120+0")

        self.stage = "ready"

        self.label = tk.Label(self.root,
                              text="Delay Training\n\nPress S to Start",
                              font=("Arial", 20),
                              justify="center")
        self.label.pack(expand=True)

        self.root.bind("<Key>", self.key_handler)

    # ----------------------------------------
    def image_callback(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_image = cv2.resize(cv_image, (1000, 700))

        cv2.imshow("Camera Feed", cv_image)
        cv2.moveWindow("Camera Feed", 0, 0)
        cv2.waitKey(1)

    # ----------------------------------------
    def key_handler(self, event):

        if self.stage == "ready" and event.char.lower() == 's':
            self.start_trial()

        elif self.stage == "phase":

            if event.keysym == 'space':
                self.next_phase()

            # ✅ Repeat allowed ONLY after Phase 2 appears
            elif event.char.lower() == 'r' and self.current_phase == 2:
                self.repeat_trial()

        elif self.stage == "complete" and event.char.lower() == 'r':
            self.repeat_trial()

    # ----------------------------------------
    def apply_delay(self, m, s):

        subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
                        shell=True)

        subprocess.call(
            f"sudo tc qdisc add dev {INTERFACE} root netem delay {m}ms",
            shell=True)

        msg = Int32MultiArray()
        msg.data = [m, s]
        self.pub.publish(msg)

    # ----------------------------------------
    def start_trial(self):

        zero = (0, 0)
        asym = TRAINING_DELAY

        # Shuffle order
        if random.random() > 0.5:
            self.phases = [zero, asym]
        else:
            self.phases = [asym, zero]

        self.current_phase = 0
        self.run_phase()

    # ----------------------------------------
    def run_phase(self):

        # ✅ Training complete
        if self.current_phase >= 2:
            self.stage = "complete"
            self.label.config(
                text="Training Complete\n\nPress R to repeat"
            )
            return

        self.stage = "phase"

        m, s = self.phases[self.current_phase]
        self.apply_delay(m, s)

        # Info text
        if (m, s) == (0, 0):
            info = "NO DELAY (Reference)"
        else:
            info = "DELAY CONDITION"

        # ✅ Phase-specific UI
        if self.current_phase == 0:
            # Phase 1
            extra = "\n\nPress SPACE to continue"
        else:
            # Phase 2
            extra = "\n\nPress SPACE to continue\nPress R to repeat BOTH phases"

        self.label.config(
            text=f"Phase {self.current_phase + 1}\n{info}{extra}"
        )

        self.current_phase += 1

    # ----------------------------------------
    def next_phase(self):
        self.run_phase()

    # ----------------------------------------
    def repeat_trial(self):

        self.current_phase = 0
        self.label.config(text="Repeating...\nStarting Phase 1")
        self.run_phase()

    # ----------------------------------------
    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    TrainingExperiment().run()