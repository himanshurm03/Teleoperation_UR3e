#!/usr/bin/env python3


#This is generating and replacating the delay perfectly just need to change the interface


# import rospy
# from std_msgs.msg import Int32
# import tkinter as tk
# import random
# import subprocess
# import csv
# import os
# from datetime import datetime

# INTERFACE = "enp2s0"

# class MasterDelayExperiment:

#     def __init__(self):

#         rospy.init_node('master_delay_node')
#         self.pub = rospy.Publisher('/delay_cmd', Int32, queue_size=10)

#         self.root = tk.Tk()
#         self.root.title("Force Perception Experiment")
#         self.root.geometry("900x600")  # Bigger window

#         self.stage = "name"  # name → ready → trial → rating
#         self.username = ""
#         self.current_delay = 0
#         self.trial_number = 0

#         self.label = tk.Label(self.root,
#                               text="Enter Your Name and Press ENTER",
#                               font=("Arial", 22),
#                               wraplength=800,
#                               justify="center")
#         self.label.pack(expand=True)

#         self.entry = tk.Entry(self.root, font=("Arial", 18))
#         self.entry.pack()
#         self.entry.focus()

#         self.root.bind("<Return>", self.handle_enter)
#         self.root.bind("<Key>", self.key_handler)

    
#     def handle_enter(self, event):

#         if self.stage == "name":
#             self.username = self.entry.get().strip()
#             if self.username == "":
#                 return

#             self.entry.pack_forget()
#             self.stage = "ready"

#             self.label.config(
#                 text=f"Welcome {self.username}\n\n"
#                      "Press 'S' to Start the Experiment"
#             )

    
#     def key_handler(self, event):

#         if self.stage == "ready":
#             if event.char.lower() == 's':
#                 self.start_trial()

#         elif self.stage == "trial":
#             # Wait for user to finish task and rate
#             if event.char.isdigit():
#                 rating = int(event.char)
#                 if 1 <= rating <= 9:
#                     self.save_trial(rating)
#                     self.start_trial()

    
#     def start_trial(self):

#         self.stage = "trial"
#         self.trial_number += 1

#         # self.current_delay = random.randint(0, 10)
#         self.current_delay = random.randint(0, 1)

#         # Remove old delay
#         subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root",
#                         shell=True)

#         # Apply new delay
#         subprocess.call(
#             f"sudo tc qdisc add dev {INTERFACE} root netem delay {self.current_delay}ms",
#             shell=True)

#         # Publish to slave
#         self.pub.publish(self.current_delay)

#         self.label.config(
#             text=f"Trial {self.trial_number}\n\n"
#                  "Perform the Teleoperation Task\n\n"
#                  "After completion,\n"
#                  "Rate Similarity (1–9)\n\n"
#                  "1 = Completely Different\n"
#                  "9 = Exactly Same"
#         )

    
#     def save_trial(self, rating):

#         filename = "/home/autonomous-lab/Desktop/delay/force_perception_results.csv"
#         file_exists = os.path.isfile(filename)

#         with open(filename, mode='a', newline='') as file:
#             writer = csv.writer(file)

#             if not file_exists:
#                 writer.writerow(["Timestamp",
#                                  "User",
#                                  "Trial",
#                                  "Delay(ms)",
#                                  "SimilarityRating"])

#             writer.writerow([
#                 datetime.now(),
#                 self.username,
#                 self.trial_number,
#                 self.current_delay,
#                 rating
#             ])

#         self.label.config(
#             text="Response Recorded.\n\nStarting Next Trial..."
#         )

#         self.root.update()
#         rospy.sleep(1.5) 


#     def run(self):
#         self.root.mainloop()


# if __name__ == "__main__":
#     gui = MasterDelayExperiment()
#     gui.run()



##########################################################################
#Symmetrical delays

# import rospy
# from std_msgs.msg import Int32
# import tkinter as tk
# import random
# import subprocess
# import csv
# import os
# from datetime import datetime

# INTERFACE = "enp2s0"
# REFERENCE_TIME = 30     # seconds
# TEST_TIME = 30          # seconds


# class DelayExperiment:

#     def __init__(self):

#         rospy.init_node('master_delay_node')
#         self.pub = rospy.Publisher('/delay_cmd', Int32, queue_size=10)

#         self.root = tk.Tk()
#         self.root.title("Force Perception Study")
#         self.root.geometry("1000x650")

#         self.stage = "name"
#         self.username = ""
#         self.trial = 0
#         self.current_delay = 0

#         self.label = tk.Label(self.root,
#                               text="Enter Your Name and Press ENTER",
#                               font=("Arial", 24),
#                               wraplength=900,
#                               justify="center")
#         self.label.pack(expand=True)

#         self.entry = tk.Entry(self.root, font=("Arial", 20))
#         self.entry.pack()
#         self.entry.focus()

#         self.root.bind("<Return>", self.handle_enter)
#         self.root.bind("<Key>", self.key_handler)

#     # ----------------------------------------------------
#     def handle_enter(self, event):
#         if self.stage == "name":
#             self.username = self.entry.get().strip()
#             if self.username == "":
#                 return
#             self.entry.pack_forget()
#             self.stage = "ready"
#             self.label.config(
#                 text=f"Welcome {self.username}\n\n"
#                      "Press 'S' to Start Trial"
#             )

#     # ----------------------------------------------------
#     def key_handler(self, event):

#         if self.stage == "ready":
#             if event.char.lower() == 's':
#                 self.start_reference_phase()

#         elif self.stage == "judgment":

#             if event.char.lower() in ['y', 'n']:
#                 self.similar_response = event.char.upper()
#                 self.label.config(
#                     text="Rate Similarity (1–9)\n\n"
#                          "1 = Completely Different\n"
#                          "9 = Exactly Same"
#                 )
#                 self.stage = "rating"

#         elif self.stage == "rating":
#             if event.char.isdigit():
#                 rating = int(event.char)
#                 if 1 <= rating <= 9:
#                     self.save_trial(rating)
#                     self.label.config(
#                         text="Response Recorded.\n\nPress 'S' for Next Trial"
#                     )
#                     self.stage = "ready"

#     # ----------------------------------------------------
#     def apply_delay(self, delay):

#         subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root",
#                         shell=True)

#         subprocess.call(
#             f"sudo tc qdisc add dev {INTERFACE} root netem delay {delay}ms",
#             shell=True)

#         self.pub.publish(delay)

#     # ----------------------------------------------------
#     def start_reference_phase(self):

#         self.trial += 1
#         self.stage = "reference"

#         self.apply_delay(0)

#         self.label.config(
#             text=f"Trial {self.trial}\n\n"
#                  "PHASE 1: Reference (No Delay)\n\n"
#                  "Explore the Environment\n\n"
#                  f"Time Remaining: {REFERENCE_TIME} seconds"
#         )

#         self.countdown(REFERENCE_TIME, self.start_test_phase)

#     # ----------------------------------------------------
#     def start_test_phase(self):

#         self.stage = "test"
#         self.current_delay = random.randint(0, 10)

#         self.apply_delay(self.current_delay)

#         self.label.config(
#             text=f"Trial {self.trial}\n\n"
#                  "PHASE 2: Test Condition\n\n"
#                  "Explore the Environment\n\n"
#                  f"Time Remaining: {TEST_TIME} seconds"
#         )

#         self.countdown(TEST_TIME, self.ask_judgment)

   
#     def ask_judgment(self):

#         self.stage = "judgment"

#         self.label.config(
#             text="Are the Two Conditions Similar?\n\n"
#                  "Press Y = Yes\n"
#                  "Press N = No"
#         )

    
#     def countdown(self, remaining, callback):

#         if remaining <= 0:
#             callback()
#             return

#         self.label.config(
#             text=self.label.cget("text").rsplit("Time Remaining:", 1)[0] +
#                  f"Time Remaining: {remaining} seconds"
#         )

#         self.root.after(1000, self.countdown, remaining - 1, callback)

    
#     def save_trial(self, rating):

#         filename = "/home/autonomous-lab/Desktop/delay/force_perception_results.csv"
#         file_exists = os.path.isfile(filename)

#         with open(filename, mode='a', newline='') as file:
#             writer = csv.writer(file)

#             if not file_exists:
#                 writer.writerow(["Timestamp",
#                                  "User",
#                                  "Trial",
#                                  "TestDelay(ms)",
#                                  "Similar(Y/N)",
#                                  "SimilarityRating"])

#             writer.writerow([
#                 datetime.now(),
#                 self.username,
#                 self.trial,
#                 self.current_delay,
#                 self.similar_response,
#                 rating
#             ])

    
#     def run(self):
#         self.root.mainloop()


# if __name__ == "__main__":
#     experiment = DelayExperiment()
#     experiment.run()




#######################################################################################

#Asymmetrical delays


# import rospy
# from std_msgs.msg import Int32MultiArray
# import tkinter as tk
# import random
# import subprocess
# import csv
# import os
# from datetime import datetime

# INTERFACE = "enp2s0"
# TOTAL_DELAY = 12
# PHASE_TIME = 30


# class DelayExperiment:

#     def __init__(self):

#         rospy.init_node('master_delay_node')
#         self.pub = rospy.Publisher('/delay_cmd', Int32MultiArray, queue_size=10)

#         self.root = tk.Tk()
#         self.root.title("Force Perception Study")
#         self.root.geometry("1000x650")

#         self.stage = "name"
#         self.trial = 0

#         self.label = tk.Label(self.root,
#                               text="Enter Name and Press ENTER",
#                               font=("Arial", 24),
#                               wraplength=900)
#         self.label.pack(expand=True)

#         self.entry = tk.Entry(self.root, font=("Arial", 20))
#         self.entry.pack()
#         self.entry.focus()

#         self.root.bind("<Return>", self.handle_enter)
#         self.root.bind("<Key>", self.key_handler)

#     # ------------------------------------------
#     def handle_enter(self, event):
#         if self.stage == "name":
#             self.username = self.entry.get().strip()
#             if self.username == "":
#                 return

#             # timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
#             # self.filename = f"{self.username}_{timestamp}.csv"

#             timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
#             base_path = "/home/autonomous-lab/Desktop/delay"
#             os.makedirs(base_path, exist_ok=True)
#             self.filename = os.path.join(base_path,
#                                          f"{self.username}_{timestamp}.csv")

#             self.entry.pack_forget()
#             self.stage = "ready"
#             self.label.config(text="Press 'S' to Start Trial")

#     # ------------------------------------------
#     def key_handler(self, event):

#         if self.stage == "ready" and event.char.lower() == 's':
#             self.start_phase1()

#         elif self.stage == "judgment" and event.char.lower() in ['y', 'n']:
#             self.similar = event.char.upper()
#             self.stage = "rating"
#             self.label.config(text="Rate Similarity (1–9)")

#         elif self.stage == "rating" and event.char.isdigit():
#             rating = int(event.char)
#             if 1 <= rating <= 9:
#                 self.save_trial(rating)
#                 self.stage = "ready"
#                 self.label.config(text="Response Saved.\nPress 'S' for Next Trial")

    
#     def apply_master_delay(self, delay):
#         subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root", shell=True)
#         subprocess.call(
#             f"sudo tc qdisc add dev {INTERFACE} root netem delay {delay}ms",
#             shell=True)

  
#     def random_split(self):
#         m = random.randint(0, TOTAL_DELAY)
#         s = TOTAL_DELAY - m
#         return m, s

   
#     def start_phase1(self):

#         self.trial += 1

#         self.m1, self.s1 = self.random_split()

#         self.apply_master_delay(self.m1)

#         msg = Int32MultiArray()
#         msg.data = [self.m1, self.s1]
#         self.pub.publish(msg)

#         self.label.config(text="Phase 1\nExplore...")
#         self.root.after(PHASE_TIME * 1000, self.start_phase2)

    
#     def start_phase2(self):

#         while True:
#             self.m2, self.s2 = self.random_split()
#             if self.m2 != self.m1:
#                 break

#         self.apply_master_delay(self.m2)

#         msg = Int32MultiArray()
#         msg.data = [self.m2, self.s2]
#         self.pub.publish(msg)

#         self.label.config(text="Phase 2\nExplore...")
#         self.root.after(PHASE_TIME * 1000, self.ask_judgment)

    
#     def ask_judgment(self):
#         self.stage = "judgment"
#         self.label.config(text="Are the two conditions similar?\nPress Y or N")

#     # ------------------------------------------
#     def save_trial(self, rating):

#         file_exists = os.path.isfile(self.filename)

#         with open(self.filename, mode='a', newline='') as file:
#             writer = csv.writer(file)

#             if not file_exists:
#                 writer.writerow(["Timestamp", "User", "Trial",
#                                  "Phase1_Master", "Phase1_Slave",
#                                  "Phase2_Master", "Phase2_Slave",
#                                  "TotalDelay", "Similar(Y/N)", "Rating"])

#             writer.writerow([
#                 datetime.now(),
#                 self.username,
#                 self.trial,
#                 self.m1, self.s1,
#                 self.m2, self.s2,
#                 TOTAL_DELAY,
#                 self.similar,
#                 rating
#             ])

#     # ------------------------------------------
#     def run(self):
#         self.root.mainloop()


# if __name__ == "__main__":
#     DelayExperiment().run()


########################################################################################
#3AFC:Asym+Sym



# import rospy
# from std_msgs.msg import Int32MultiArray
# import tkinter as tk
# import random
# import subprocess
# import csv
# import os
# from datetime import datetime

# INTERFACE = "enp2s0"
# TOTAL_DELAY = 16   


# class Experiment:

#     def __init__(self):

#         rospy.init_node('master_delay_node')
#         self.pub = rospy.Publisher('/delay_cmd', Int32MultiArray, queue_size=10)

#         self.root = tk.Tk()
#         self.root.title("Delay Perception Experiment")
#         self.root.geometry("500x250")

#         self.stage = "name"
#         self.trial = 0

#         # Generate ALL asymmetric pairs (including reverse)
#         self.asym_pairs = self.generate_all_pairs()
#         random.shuffle(self.asym_pairs)

#         self.label = tk.Label(self.root, text="Enter Name and Press ENTER",
#                               font=("Arial", 18))
#         self.label.pack(expand=True)

#         self.entry = tk.Entry(self.root, font=("Arial", 14))
#         self.entry.pack()
#         self.entry.focus()

#         self.root.bind("<Return>", self.handle_enter)
#         self.root.bind("<Key>", self.key_handler)

#     # ----------------------------------------
#     def generate_all_pairs(self):
#         pairs = []
#         for i in range(1, TOTAL_DELAY):
#             if i != TOTAL_DELAY - i:
#                 pairs.append((i, TOTAL_DELAY - i))
#         return pairs

#     # ----------------------------------------
#     def handle_enter(self, event):
#         if self.stage == "name":
#             self.username = self.entry.get().strip()
#             if not self.username:
#                 return

#             timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

#             base_path = "/home/autonomous-lab/Desktop/delay/c/exp_16"
#             os.makedirs(base_path, exist_ok=True)

#             self.filename = os.path.join(base_path,
#                                         f"{self.username}_{timestamp}.csv")

#             self.entry.pack_forget()
#             self.stage = "ready"
#             self.label.config(text="Press S to Start Trial")

#     # ----------------------------------------
#     def key_handler(self, event):

#         # Start trial
#         if self.stage == "ready" and event.char.lower() == 's':
#             self.start_trial()

#         # SPACE → next phase
#         elif self.stage == "phase" and event.keysym == 'space':
#             self.next_phase()

#         # Answer
#         elif self.stage == "response" and event.char in ['1', '2', '3']:
#             self.user_answer = int(event.char)
#             self.save_trial()
#             self.stage = "ready"
#             self.label.config(text="Saved.\nPress S for next trial")

#     # ----------------------------------------
#     def apply_delay(self, m, s):

#         subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
#                         shell=True)

#         subprocess.call(
#             f"sudo tc qdisc add dev {INTERFACE} root netem delay {m}ms",
#             shell=True)

#         msg = Int32MultiArray()
#         msg.data = [m, s]
#         self.pub.publish(msg)

#     # ----------------------------------------
#     def start_trial(self):

#         if len(self.asym_pairs) == 0:
#             self.label.config(text="All asymmetric cases used!")
#             return

#         self.trial += 1

#         sym = (TOTAL_DELAY // 2, TOTAL_DELAY // 2)

#         # Pick unique asymmetric pair
#         self.asym_m, self.asym_s = self.asym_pairs.pop()

#         # Build phases
#         self.phases = [
#             {"type": "sym", "delay": sym},
#             {"type": "sym", "delay": sym},
#             {"type": "asym", "delay": (self.asym_m, self.asym_s)}
#         ]

#         # Shuffle phases
#         random.shuffle(self.phases)

#         # Find correct phase
#         for i, p in enumerate(self.phases):
#             if p["type"] == "asym":
#                 self.correct_phase = i + 1

#         self.current_phase = 0
#         self.run_phase()

#     # ----------------------------------------
#     def run_phase(self):

#         if self.current_phase >= 3:
#             self.ask_response()
#             return

#         self.stage = "phase"

#         phase = self.phases[self.current_phase]
#         m, s = phase["delay"]

#         self.apply_delay(m, s)

#         self.label.config(
#             text=f"Phase {self.current_phase + 1}\nExplore...\nPress SPACE for next phase"
#         )

#         self.current_phase += 1

#     # ----------------------------------------
#     def next_phase(self):
#         self.run_phase()

#     # ----------------------------------------
#     def ask_response(self):

#         self.stage = "response"

#         self.label.config(
#             text="Which phase is DIFFERENT?\n\nPress 1 / 2 / 3"
#         )

#     # ----------------------------------------
#     def save_trial(self):

#         file_exists = os.path.isfile(self.filename)

#         with open(self.filename, mode='a', newline='') as file:
#             writer = csv.writer(file)

#             if not file_exists:
#                 writer.writerow([
#                     "Timestamp", "User", "Trial",
#                     "TotalDelay",
#                     "Phase1", "Phase2", "Phase3",
#                     "Asym_Master", "Asym_Slave",
#                     "CorrectPhase", "UserAnswer"
#                 ])

#             writer.writerow([
#                 datetime.now(),
#                 self.username,
#                 self.trial,
#                 TOTAL_DELAY,
#                 self.phases[0]["delay"],
#                 self.phases[1]["delay"],
#                 self.phases[2]["delay"],
#                 self.asym_m,
#                 self.asym_s,
#                 self.correct_phase,
#                 self.user_answer
#             ])

#     # ----------------------------------------
#     def run(self):
#         self.root.mainloop()


# if __name__ == "__main__":
#     Experiment().run()

######################################################################

#2AFC:Asymm+Symm : Camera Feedback :No repeat option

#!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Int32MultiArray
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# import tkinter as tk
# import random
# import subprocess
# import csv
# import os
# from datetime import datetime

# import cv2
# import numpy as np

# INTERFACE = "enp2s0"
# TOTAL_DELAY = 16   

# class Experiment:

#     def __init__(self):

#         rospy.init_node('master_2afc_experiment')

#         self.pub = rospy.Publisher('/delay_cmd', Int32MultiArray, queue_size=10)

#         # Camera setup
#         self.bridge = CvBridge()
#         rospy.Subscriber('/camera_feed', Image, self.image_callback)

#         # GUI
#         self.root = tk.Tk()
#         self.root.title("2AFC Delay Experiment")
#         self.root.geometry("600x400+1120+0")

#         self.stage = "name"
#         self.trial = 0

#         # Asymmetric pairs
#         self.asym_pairs = self.generate_all_pairs()
#         random.shuffle(self.asym_pairs)

#         # Balance order (sym first / asym first)
#         self.order_toggle = True

#         self.label = tk.Label(self.root, text="Enter Name and Press ENTER",
#                               font=("Arial", 20))
#         self.label.pack()

#         self.entry = tk.Entry(self.root, font=("Arial", 16))
#         self.entry.pack()
#         self.entry.focus()

#         self.root.bind("<Return>", self.handle_enter)
#         self.root.bind("<Key>", self.key_handler)

#         self.current_frame = None

#     # ----------------------------------------
#     def generate_all_pairs(self):
#         pairs = []
#         for i in range(1, TOTAL_DELAY):
#             if i != TOTAL_DELAY - i:
#                 pairs.append((i, TOTAL_DELAY - i))
#         return pairs

#     # ----------------------------------------
#     # def image_callback(self, msg):

#     #     cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#     #     cv_image = cv2.resize(cv_image, (640, 360))

#     #     self.current_frame = cv_image
#     #     cv2.imshow("Camera Feed", cv_image)
#     #     cv2.waitKey(1)
    

#     def image_callback(self, msg):

#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         cv_image = cv2.resize(cv_image, (1000, 700))  

#         cv2.imshow("Camera Feed", cv_image)

# #  Set size + position (ADD THIS)
#         cv2.resizeWindow("Camera Feed", 1100, 700)
#         cv2.moveWindow("Camera Feed", 0, 0)

#         cv2.waitKey(1)
#     # ----------------------------------------
#     def handle_enter(self, event):

#         if self.stage == "name":

#             self.username = self.entry.get().strip()
#             if not self.username:
#                 return

#             timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

#             base_path = "/home/autonomous-lab/Desktop/delay/2AFC/14c/userexp/U1"
#             os.makedirs(base_path, exist_ok=True)

#             self.filename = os.path.join(base_path,
#                                         f"{self.username}_{timestamp}.csv")

#             self.entry.pack_forget()
#             self.stage = "ready"
#             self.label.config(text="Press S to Start Trial")

#     # ----------------------------------------
#     def key_handler(self, event):

#         if self.stage == "ready" and event.char.lower() == 's':
#             self.start_trial()

#         elif self.stage == "phase" and event.keysym == 'space':
#             self.next_phase()

#         elif self.stage == "response" and event.char.lower() in ['y', 'n']:
#             self.user_answer = event.char.upper()
#             self.save_trial()
#             self.stage = "ready"
#             self.label.config(text="Saved.\nPress S for next trial")

#     # ----------------------------------------
#     def apply_delay(self, m, s):

#         subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
#                         shell=True)

#         subprocess.call(
#             f"sudo tc qdisc add dev {INTERFACE} root netem delay {m}ms",
#             shell=True)

#         msg = Int32MultiArray()
#         msg.data = [m, s]
#         self.pub.publish(msg)

#     # ----------------------------------------
#     def start_trial(self):

#         if len(self.asym_pairs) == 0:
#             self.label.config(text="All asymmetric cases used!")
#             return

#         self.trial += 1

#         sym = (TOTAL_DELAY // 2, TOTAL_DELAY // 2)
#         self.asym = self.asym_pairs.pop()

#         # Balanced order
#         if self.order_toggle:
#             self.phases = [
#                 {"type": "sym", "delay": sym},
#                 {"type": "asym", "delay": self.asym}
#             ]
#         else:
#             self.phases = [
#                 {"type": "asym", "delay": self.asym},
#                 {"type": "sym", "delay": sym}
#             ]

#         self.order_toggle = not self.order_toggle

#         self.current_phase = 0
#         self.run_phase()

#     # ----------------------------------------
#     def run_phase(self):

#         if self.current_phase >= 2:
#             self.ask_response()
#             return

#         self.stage = "phase"

#         phase = self.phases[self.current_phase]
#         m, s = phase["delay"]

#         self.apply_delay(m, s)

#         self.label.config(
#             text=f"Phase {self.current_phase + 1}\nExplore...\nPress SPACE for next"
#         )

#         self.current_phase += 1

#     # ----------------------------------------
#     def next_phase(self):
#         self.run_phase()

#     # ----------------------------------------
#     def ask_response(self):

#         self.stage = "response"

#         self.label.config(
#             text="Did both phases feel DIFFERENT?\nPress Y / N"
#         )

#     # ----------------------------------------
#     def save_trial(self):

#         file_exists = os.path.isfile(self.filename)

#         with open(self.filename, mode='a', newline='') as file:
#             writer = csv.writer(file)

#             if not file_exists:
#                 writer.writerow([
#                     "Timestamp", "User", "Trial",
#                     "TotalDelay",
#                     "Phase1", "Phase2",
#                     "AsymPair",
#                     "UserAnswer"
#                 ])

#             writer.writerow([
#                 datetime.now(),
#                 self.username,
#                 self.trial,
#                 TOTAL_DELAY,
#                 self.phases[0]["delay"],
#                 self.phases[1]["delay"],
#                 self.asym,
#                 self.user_answer
#             ])

#     # ----------------------------------------
#     def run(self):
#         self.root.mainloop()


# if __name__ == "__main__":
#     Experiment().run()

#########################################################################


#2AFC:Asymm+Symm : Camera Feedback :Repeat Option

# !/usr/bin/env python3

# import rospy
# from std_msgs.msg import Int32MultiArray
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# import tkinter as tk
# import random
# import subprocess
# import csv
# import os
# from datetime import datetime

# import cv2

# INTERFACE = "enp2s0"
# TOTAL_DELAY = 16


# class Experiment:

#     def __init__(self):

#         rospy.init_node('master_2afc_experiment')
#         self.pub = rospy.Publisher('/delay_cmd', Int32MultiArray, queue_size=10)

#         # Camera
#         self.bridge = CvBridge()
#         rospy.Subscriber('/camera_feed', Image, self.image_callback)

#         # GUI
#         self.root = tk.Tk()
#         self.root.title("2AFC Delay Experiment")
#         self.root.geometry("600x400+1120+0")

#         self.stage = "name"
#         self.trial = 0

#         self.asym_pairs = self.generate_all_pairs()
#         random.shuffle(self.asym_pairs)

#         self.order_toggle = True

#         self.label = tk.Label(self.root, text="Enter Name and Press ENTER",
#                               font=("Arial", 20))
#         self.label.pack()

#         self.entry = tk.Entry(self.root, font=("Arial", 16))
#         self.entry.pack()
#         self.entry.focus()

#         self.root.bind("<Return>", self.handle_enter)
#         self.root.bind("<Key>", self.key_handler)

#     # ----------------------------------------
#     def generate_all_pairs(self):
#         pairs = []
#         for i in range(1, TOTAL_DELAY):
#             if i != TOTAL_DELAY - i:
#                 pairs.append((i, TOTAL_DELAY - i))
#         return pairs

#     # ----------------------------------------
#     def image_callback(self, msg):

#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         cv_image = cv2.resize(cv_image, (1000, 700))

#         cv2.imshow("Camera Feed", cv_image)
#         cv2.resizeWindow("Camera Feed", 1100, 700)
#         cv2.moveWindow("Camera Feed", 0, 0)
#         cv2.waitKey(1)

#     # ----------------------------------------
#     def handle_enter(self, event):

#         if self.stage == "name":

#             self.username = self.entry.get().strip()
#             if not self.username:
#                 return

#             timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

#             base_path = "/home/autonomous-lab/Desktop/delay/2AFC/14c/userexp/U1"
#             os.makedirs(base_path, exist_ok=True)

#             self.filename = os.path.join(base_path,
#                                         f"{self.username}_{timestamp}.csv")

#             self.entry.pack_forget()
#             self.stage = "ready"
#             self.label.config(text="Press S to Start Trial")

#     # ----------------------------------------
#     def key_handler(self, event):

#         if self.stage == "ready" and event.char.lower() == 's':
#             self.start_trial()

#         elif self.stage == "phase":

#             # SPACE → next phase
#             if event.keysym == 'space':
#                 self.next_phase()

            
#             elif event.char.lower() == 'r' and self.current_phase == 2:
#                 self.repeat_trial()

#         elif self.stage == "response" and event.char.lower() in ['y', 'n']:
#             self.user_answer = event.char.upper()
#             self.save_trial()
#             self.stage = "ready"
#             self.label.config(text="Saved.\nPress S for next trial")

#     # ----------------------------------------
#     def apply_delay(self, m, s):

#         subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
#                         shell=True)

#         subprocess.call(
#             f"sudo tc qdisc add dev {INTERFACE} root netem delay {m}ms",
#             shell=True)

#         msg = Int32MultiArray()
#         msg.data = [m, s]
#         self.pub.publish(msg)

#     # ----------------------------------------
#     def start_trial(self):

#         if len(self.asym_pairs) == 0:
#             self.label.config(text="All asymmetric cases used!")
#             return

#         self.trial += 1

#         sym = (TOTAL_DELAY // 2, TOTAL_DELAY // 2)
#         self.asym = self.asym_pairs.pop()

#         if self.order_toggle:
#             self.phases = [
#                 {"type": "sym", "delay": sym},
#                 {"type": "asym", "delay": self.asym}
#             ]
#         else:
#             self.phases = [
#                 {"type": "asym", "delay": self.asym},
#                 {"type": "sym", "delay": sym}
#             ]

#         self.order_toggle = not self.order_toggle

#         self.current_phase = 0
#         self.run_phase()

#     # ----------------------------------------
#     def run_phase(self):

#         if self.current_phase >= 2:
#             self.ask_response()
#             return

#         self.stage = "phase"

#         phase = self.phases[self.current_phase]
#         m, s = phase["delay"]

#         self.apply_delay(m, s)

#         # Show instruction
#         if self.current_phase == 0:
#             extra = "\nPress SPACE to continue"
#         else:
#             extra = "\nPress SPACE to continue | Press R to repeat"

#         self.label.config(
#             text=f"Phase {self.current_phase + 1}\nExplore...{extra}"
#         )

#         self.current_phase += 1

#     # ----------------------------------------
#     def repeat_phase(self):

#         phase_index = self.current_phase - 1
#         phase = self.phases[phase_index]

#         m, s = phase["delay"]

#         self.apply_delay(m, s)

#         self.label.config(
#             text=f"Phase {phase_index + 1} (REPEAT)\nExplore...\nPress SPACE to continue"
#         )

#     # ----------------------------------------
#     def next_phase(self):
#         self.run_phase()


#     def repeat_trial(self):

#     # Reset to beginning of trial
#         self.current_phase = 0

#         self.label.config(
#             text="Repeating both phases...\nStarting Phase 1"
#         )

#     # Restart phases with SAME delays
#         self.run_phase()

#     # ----------------------------------------
#     def ask_response(self):

#         self.stage = "response"

#         self.label.config(
#             text="Did both phases feel DIFFERENT?\nPress Y / N"
#         )

#     # ----------------------------------------
#     def save_trial(self):

#         file_exists = os.path.isfile(self.filename)

#         with open(self.filename, mode='a', newline='') as file:
#             writer = csv.writer(file)

#             if not file_exists:
#                 writer.writerow([
#                     "Timestamp", "User", "Trial",
#                     "TotalDelay",
#                     "Phase1", "Phase2",
#                     "AsymPair",
#                     "UserAnswer"
#                 ])

#             writer.writerow([
#                 datetime.now(),
#                 self.username,
#                 self.trial,
#                 TOTAL_DELAY,
#                 self.phases[0]["delay"],
#                 self.phases[1]["delay"],
#                 self.asym,
#                 self.user_answer
#             ])

#     # ----------------------------------------
#     def run(self):
#         self.root.mainloop()


# if __name__ == "__main__":
#     Experiment().run()



#########################################################################

#Dissimilarity Rating :Final
#Delay


#!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Int32MultiArray
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# import tkinter as tk
# import random
# import subprocess
# import csv
# import os
# from datetime import datetime

# import cv2

# INTERFACE = "enp2s0"
# TOTAL_DELAY = 16


# class Experiment:

#     def __init__(self):

#         rospy.init_node('master_rating_experiment')
#         self.pub = rospy.Publisher('/delay_cmd', Int32MultiArray, queue_size=10)

#         # Camera
#         self.bridge = CvBridge()
#         rospy.Subscriber('/camera_feed', Image, self.image_callback)

#         # GUI
#         self.root = tk.Tk()
#         self.root.title("Delay Rating Experiment")
#         self.root.geometry("800x400+1120+0")

#         self.stage = "name"
#         self.trial = 0

        
#         self.asym_pairs = [
#             (1,15), (15,1),
#             (3,13), (13,3),
#             (5,11), (11,5)
#         ]
#         random.shuffle(self.asym_pairs)

#         self.order_toggle = True

#         self.label = tk.Label(self.root, text="Enter Name and Press ENTER",
#                               font=("Arial", 20))
#         self.label.pack()

#         self.entry = tk.Entry(self.root, font=("Arial", 16))
#         self.entry.pack()
#         self.entry.focus()

#         self.root.bind("<Return>", self.handle_enter)
#         self.root.bind("<Key>", self.key_handler)

#     # ----------------------------------------
#     def image_callback(self, msg):

#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         cv_image = cv2.resize(cv_image, (1000, 700))

#         cv2.imshow("Camera Feed", cv_image)
#         cv2.resizeWindow("Camera Feed", 1100, 700)
#         cv2.moveWindow("Camera Feed", 0, 0)
#         cv2.waitKey(1)

#     # ----------------------------------------
#     def handle_enter(self, event):

#         if self.stage == "name":

#             self.username = self.entry.get().strip()
#             if not self.username:
#                 return

#             timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

#             base_path = "/home/autonomous-lab/Desktop/delay/DS"
#             os.makedirs(base_path, exist_ok=True)

#             self.filename = os.path.join(base_path,
#                                         f"{self.username}_{timestamp}.csv")

#             self.entry.pack_forget()
#             self.stage = "ready"
#             self.label.config(text="Press S to Start Trial")

#     # ----------------------------------------
#     def key_handler(self, event):

#         if self.stage == "ready" and event.char.lower() == 's':
#             self.start_trial()

#         elif self.stage == "phase":

#             if event.keysym == 'space':
#                 self.next_phase()

#             elif event.char.lower() == 'r' and self.current_phase == 2:
#                 self.repeat_trial()

        
#         elif self.stage == "response":

#             if event.char.isdigit():
#                 rating = int(event.char)
 
#                 if 0 <= rating <= 9:
#                     self.user_rating = rating
#                     self.save_trial()
#                     self.stage = "ready"
#                     self.label.config(text="Saved.\nPress S for next trial")

#             elif event.char.lower() == 't':
#                 self.user_rating = 10
#                 self.save_trial()
#                 self.stage = "ready"
#                 self.label.config(text="Saved.\nPress S for next trial")

#     # ----------------------------------------
#     def apply_delay(self, m, s):

#         subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
#                         shell=True)

#         subprocess.call(
#             f"sudo tc qdisc add dev {INTERFACE} root netem delay {m}ms",
#             shell=True)

#         msg = Int32MultiArray()
#         msg.data = [m, s]
#         self.pub.publish(msg)

#     # ----------------------------------------
#     def start_trial(self):

#         if len(self.asym_pairs) == 0:
#             self.label.config(text="All trials completed!")
#             return

#         self.trial += 1

#         sym = (8, 8)
#         self.asym = self.asym_pairs.pop()

#         if self.order_toggle:
#             self.phases = [
#                 {"type": "sym", "delay": sym},
#                 {"type": "asym", "delay": self.asym}
#             ]
#         else:
#             self.phases = [
#                 {"type": "asym", "delay": self.asym},
#                 {"type": "sym", "delay": sym}
#             ]

#         self.order_toggle = not self.order_toggle

#         self.current_phase = 0
#         #self.pending_one = False
#         self.run_phase()

#     # ----------------------------------------
#     def run_phase(self):

#         if self.current_phase >= 2:
#             self.ask_response()
#             return

#         self.stage = "phase"

#         phase = self.phases[self.current_phase]
#         m, s = phase["delay"]

#         self.apply_delay(m, s)

#         if self.current_phase == 0:
#             extra = "\nPress SPACE to continue"
#         else:
#             extra = "\nPress SPACE to continue | Press R to repeat BOTH phases"

#         self.label.config(
#             text=f"Phase {self.current_phase + 1}\nExplore...{extra}"
#         )

#         self.current_phase += 1

#     # ----------------------------------------
#     def repeat_trial(self):

#         self.current_phase = 0
#         self.label.config(text="Repeating both phases...\nStarting Phase 1")
#         self.run_phase()

#     # ----------------------------------------
#     def next_phase(self):
#         self.run_phase()

#     # ----------------------------------------
#     def ask_response(self):

#         self.stage = "response"

#         self.label.config(
#             text="Rate Dissimilarity (0–10)\n\n"
#                  "0 = Same\n"
#                  "10 = Completely different\n\n"
#                  "Press 0–9 or T for 10"
                 
#         )

#     # ----------------------------------------
#     def save_trial(self):

#         file_exists = os.path.isfile(self.filename)

#         with open(self.filename, mode='a', newline='') as file:
#             writer = csv.writer(file)

#             if not file_exists:
#                 writer.writerow([
#                     "Timestamp", "User", "Trial",
#                     "Phase1", "Phase2",
#                     "AsymPair",
#                     "Rating"
#                 ])

#             writer.writerow([
#                 datetime.now(),
#                 self.username,
#                 self.trial,
#                 self.phases[0]["delay"],
#                 self.phases[1]["delay"],
#                 self.asym,
#                 self.user_rating
#             ])

#     # ----------------------------------------
#     def run(self):
#         self.root.mainloop()


# if __name__ == "__main__":
#     Experiment().run()


######################################################################

#Packet loss
#10%

# !/usr/bin/env python3

# import rospy
# from std_msgs.msg import Int32MultiArray
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# import tkinter as tk
# import random
# import subprocess
# import csv
# import os
# from datetime import datetime

# import cv2

# INTERFACE = "enp2s0"
# TOTAL_LOSS = 10


# class Experiment:

#     def __init__(self):

#         rospy.init_node('master_packetloss_experiment')
#         self.pub = rospy.Publisher('/loss_cmd', Int32MultiArray, queue_size=10)

#         # Camera
#         self.bridge = CvBridge()
#         rospy.Subscriber('/camera_feed', Image, self.image_callback)

#         # GUI
#         self.root = tk.Tk()
#         self.root.title("Packet Loss Rating Experiment")
#         self.root.geometry("700x450+1100+0")

#         self.stage = "name"
#         self.trial = 0

#         self.asym_pairs = self.generate_all_pairs()
#         random.shuffle(self.asym_pairs)

#         self.order_toggle = True

#         self.label = tk.Label(self.root,
#                               text="Enter Name and Press ENTER",
#                               font=("Arial", 20),
#                               wraplength=650,
#                               justify="center")
#         self.label.pack()

#         self.entry = tk.Entry(self.root, font=("Arial", 16))
#         self.entry.pack()
#         self.entry.focus()

#         self.root.bind("<Return>", self.handle_enter)
#         self.root.bind("<Key>", self.key_handler)

#     # ----------------------------------------
#     def generate_all_pairs(self):
#         pairs = []
#         for i in range(1, TOTAL_LOSS):
#             if i != TOTAL_LOSS - i:
#                 pairs.append((i, TOTAL_LOSS - i))
#         return pairs

#     # ----------------------------------------
#     def image_callback(self, msg):

#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         cv_image = cv2.resize(cv_image, (1000, 700))

#         cv2.imshow("Camera Feed", cv_image)
#         cv2.resizeWindow("Camera Feed", 1100, 700)
#         cv2.moveWindow("Camera Feed", 0, 0)
#         cv2.waitKey(1)

#     # ----------------------------------------
#     def handle_enter(self, event):

#         if self.stage == "name":

#             self.username = self.entry.get().strip()
#             if not self.username:
#                 return

#             timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

#             base_path = "/home/autonomous-lab/Desktop/delay/Packetloss"
#             os.makedirs(base_path, exist_ok=True)

#             self.filename = os.path.join(base_path,
#                                         f"{self.username}_{timestamp}.csv")

#             self.entry.pack_forget()
#             self.stage = "ready"
#             self.label.config(text="Press S to Start Trial")

#     # ----------------------------------------
#     def key_handler(self, event):

#         if self.stage == "ready" and event.char.lower() == 's':
#             self.start_trial()

#         elif self.stage == "phase":

#             if event.keysym == 'space':
#                 self.next_phase()

#             elif event.char.lower() == 'r' and self.current_phase == 2:
#                 self.repeat_trial()

#         elif self.stage == "response":

#             if event.char.isdigit():
#                 rating = int(event.char)

#                 # Handle "10"
#                 if rating == 1 and self.pending_one:
#                     rating = 10
#                     self.pending_one = False
#                 elif rating == 1:
#                     self.pending_one = True
#                     return
#                 else:
#                     self.pending_one = False

#                 if 0 <= rating <= 10:
#                     self.user_rating = rating
#                     self.save_trial()
#                     self.stage = "ready"
#                     self.label.config(text="Saved.\nPress S for next trial")

#     # ----------------------------------------
#     def apply_loss(self, m, s):

#         subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
#                         shell=True)

#         subprocess.call(
#             f"sudo tc qdisc add dev {INTERFACE} root netem loss {m}%",
#             shell=True)

#         msg = Int32MultiArray()
#         msg.data = [m, s]
#         self.pub.publish(msg)

#     # ----------------------------------------
#     def start_trial(self):

#         if len(self.asym_pairs) == 0:
#             self.label.config(text="All asymmetric cases used!")
#             return

#         self.trial += 1

#         sym = (TOTAL_LOSS // 2, TOTAL_LOSS // 2)
#         self.asym = self.asym_pairs.pop()

#         if self.order_toggle:
#             self.phases = [
#                 {"type": "sym", "loss": sym},
#                 {"type": "asym", "loss": self.asym}
#             ]
#         else:
#             self.phases = [
#                 {"type": "asym", "loss": self.asym},
#                 {"type": "sym", "loss": sym}
#             ]

#         self.order_toggle = not self.order_toggle

#         self.current_phase = 0
#         self.pending_one = False
#         self.run_phase()

#     # ----------------------------------------
#     def run_phase(self):

#         if self.current_phase >= 2:
#             self.ask_response()
#             return

#         self.stage = "phase"

#         phase = self.phases[self.current_phase]
#         m, s = phase["loss"]

#         self.apply_loss(m, s)

#         if self.current_phase == 0:
#             extra = "\nPress SPACE to continue"
#         else:
#             extra = "\nPress SPACE to continue | Press R to repeat"

#         self.label.config(
#             text=f"Phase {self.current_phase + 1}\nExplore...{extra}"
#         )

#         self.current_phase += 1

#     # ----------------------------------------
#     def repeat_trial(self):

#         self.current_phase = 0
#         self.label.config(text="Repeating both phases...\nStarting Phase 1")
#         self.run_phase()

#     # ----------------------------------------
#     def next_phase(self):
#         self.run_phase()

#     # ----------------------------------------
#     def ask_response(self):

#         self.stage = "response"

#         self.label.config(
#             text="Rate Dissimilarity (0–10)\n\n"
#                  "0 = Same\n"
#                  "10 = Completely different"
#         )

#     # ----------------------------------------
#     def save_trial(self):

#         file_exists = os.path.isfile(self.filename)

#         with open(self.filename, mode='a', newline='') as file:
#             writer = csv.writer(file)

#             if not file_exists:
#                 writer.writerow([
#                     "Timestamp", "User", "Trial",
#                     "TotalLoss",
#                     "Phase1", "Phase2",
#                     "AsymPair",
#                     "Rating"
#                 ])

#             writer.writerow([
#                 datetime.now(),
#                 self.username,
#                 self.trial,
#                 TOTAL_LOSS,
#                 self.phases[0]["loss"],
#                 self.phases[1]["loss"],
#                 self.asym,
#                 self.user_rating
#             ])

#     # ----------------------------------------
#     def run(self):
#         self.root.mainloop()


# if __name__ == "__main__":
#     Experiment().run()


####################################################################


#Packet loss 16%

#!/usr/bin/env python3


# import rospy
# from std_msgs.msg import Int32MultiArray
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# import tkinter as tk
# import random
# import subprocess
# import csv
# import os
# from datetime import datetime

# import cv2

# INTERFACE = "enp2s0"

# # ==========================================
# # TOTAL PACKET LOSS
# # ==========================================
# TOTAL_LOSS = 16

# # ==========================================
# # FIXED ASYMMETRIC COMBINATIONS
# # ==========================================
# ASYM_PAIRS = [
#     (1,15),
#     (15,1),
#     (3,13),
#     (13,3),
#     (4,12),
#     (12,4)
# ]

# # ==========================================
# # SYMMETRIC CONDITION
# # ==========================================
# SYMMETRIC = (8, 8)


# class Experiment:

#     def __init__(self):

#         rospy.init_node('master_packetloss_experiment')

#         self.pub = rospy.Publisher(
#             '/loss_cmd',
#             Int32MultiArray,
#             queue_size=10
#         )

#         # ----------------------------------
#         # Camera
#         # ----------------------------------
#         self.bridge = CvBridge()

#         rospy.Subscriber(
#             '/camera_feed',
#             Image,
#             self.image_callback
#         )

#         # ----------------------------------
#         # GUI
#         # ----------------------------------
#         self.root = tk.Tk()

#         self.root.title("Packet Loss Rating Experiment")

#         self.root.geometry("700x450+1100+0")

#         self.stage = "name"

#         self.trial = 0

#         # ----------------------------------
#         # Trial list
#         # ----------------------------------
#         self.trials = []

#         # Symmetric vs symmetric trial
#         self.trials.append({
#             "type": "sym_sym",
#             "pair": None
#         })

#         # Symmetric vs asymmetric trials
#         for pair in ASYM_PAIRS:
#             self.trials.append({
#                 "type": "asym",
#                 "pair": pair
#             })

#         random.shuffle(self.trials)

#         self.order_toggle = True

#         # ----------------------------------
#         # GUI Label
#         # ----------------------------------
#         self.label = tk.Label(
#             self.root,
#             text="Enter Name and Press ENTER",
#             font=("Arial", 22),
#             wraplength=750,
#             justify="center"
#         )

#         self.label.pack(expand=True)

#         # ----------------------------------
#         # Entry
#         # ----------------------------------
#         self.entry = tk.Entry(
#             self.root,
#             font=("Arial", 18)
#         )

#         self.entry.pack()

#         self.entry.focus()

#         # ----------------------------------
#         # Keyboard Bindings
#         # ----------------------------------
#         self.root.bind("<Return>", self.handle_enter)

#         self.root.bind("<Key>", self.key_handler)

#     # ======================================
#     # Camera Callback
#     # ======================================
#     def image_callback(self, msg):

#         cv_image = self.bridge.imgmsg_to_cv2(
#             msg,
#             desired_encoding='passthrough'
#         )

#         cv_image = cv2.resize(cv_image, (1000, 700))

#         cv2.imshow("Camera Feed", cv_image)

#         cv2.resizeWindow("Camera Feed", 1100, 700)

#         cv2.moveWindow("Camera Feed", 0, 0)

#         cv2.waitKey(1)

#     # ======================================
#     # Name Entry
#     # ======================================
#     def handle_enter(self, event):

#         if self.stage == "name":

#             self.username = self.entry.get().strip()

#             if not self.username:
#                 return

#             timestamp = datetime.now().strftime(
#                 "%Y-%m-%d_%H-%M-%S"
#             )

#             base_path = "/home/autonomous-lab/Desktop/delay/Packetloss"

#             os.makedirs(base_path, exist_ok=True)

#             self.filename = os.path.join(
#                 base_path,
#                 f"{self.username}_{timestamp}.csv"
#             )

#             self.entry.pack_forget()

#             self.stage = "ready"

#             self.label.config(
#                 text="Press S to Start Trial"
#             )

#     # ======================================
#     # Keyboard Handler
#     # ======================================
#     def key_handler(self, event):

#         # ----------------------------------
#         # Start Trial
#         # ----------------------------------
#         if self.stage == "ready":

#             if event.char.lower() == 's':
#                 self.start_trial()

#         # ----------------------------------
#         # During Phase
#         # ----------------------------------
#         elif self.stage == "phase":

#             # SPACE → next phase
#             if event.keysym == 'space':
#                 self.next_phase()

#             # R → repeat both phases
#             elif (
#                 event.char.lower() == 'r'
#                 and self.current_phase == 2
#             ):
#                 self.repeat_trial()

#         # ----------------------------------
#         # Rating Input
#         # ----------------------------------
#         elif self.stage == "response":

#             if event.char.isdigit():

#                 rating = int(event.char)

#                 # Handle 10
#                 if rating == 1 and self.pending_one:

#                     rating = 10

#                     self.pending_one = False

#                 elif rating == 1:

#                     self.pending_one = True
#                     return

#                 else:
#                     self.pending_one = False

#                 if 0 <= rating <= 10:

#                     self.user_rating = rating

#                     self.save_trial()

#                     self.stage = "ready"

#                     self.label.config(
#                         text="Saved.\nPress S for next trial"
#                     )

#     # ======================================
#     # Apply Packet Loss
#     # ======================================
#     def apply_loss(self, master_loss, slave_loss):

#         # Clear previous packet loss
#         subprocess.call(
#             f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
#             shell=True
#         )

#         # Apply MASTER packet loss
#         subprocess.call(
#             f"sudo tc qdisc add dev {INTERFACE} "
#             f"root netem loss {master_loss}%",
#             shell=True
#         )

#         # Send SLAVE packet loss
#         msg = Int32MultiArray()

#         msg.data = [
#             int(master_loss),
#             int(slave_loss)
#         ]

#         self.pub.publish(msg)

#     # ======================================
#     # Start Trial
#     # ======================================
#     def start_trial(self):

#         if len(self.trials) == 0:

#             self.label.config(
#                 text="All trials completed!"
#             )

#             return

#         self.trial += 1

#         trial_data = self.trials.pop()

#         # ----------------------------------
#         # Symmetric vs Symmetric
#         # ----------------------------------
#         if trial_data["type"] == "sym_sym":

#             self.asym_pair = SYMMETRIC

#             self.phases = [
#                 {
#                     "type": "sym",
#                     "loss": SYMMETRIC
#                 },
#                 {
#                     "type": "sym",
#                     "loss": SYMMETRIC
#                 }
#             ]

#         # ----------------------------------
#         # Symmetric vs Asymmetric
#         # ----------------------------------
#         else:

#             self.asym_pair = trial_data["pair"]

#             if self.order_toggle:

#                 self.phases = [
#                     {
#                         "type": "sym",
#                         "loss": SYMMETRIC
#                     },
#                     {
#                         "type": "asym",
#                         "loss": self.asym_pair
#                     }
#                 ]

#             else:

#                 self.phases = [
#                     {
#                         "type": "asym",
#                         "loss": self.asym_pair
#                     },
#                     {
#                         "type": "sym",
#                         "loss": SYMMETRIC
#                     }
#                 ]

#             self.order_toggle = not self.order_toggle

#         random.shuffle(self.phases)

#         self.current_phase = 0

#         self.pending_one = False

#         self.run_phase()

#     # ======================================
#     # Run Phase
#     # ======================================
#     def run_phase(self):

#         if self.current_phase >= 2:

#             self.ask_response()

#             return

#         self.stage = "phase"

#         phase = self.phases[self.current_phase]

#         master_loss, slave_loss = phase["loss"]

#         self.apply_loss(
#             master_loss,
#             slave_loss
#         )

#         # UI Text
#         if self.current_phase == 0:

#             extra = "\nPress SPACE to continue"

#         else:

#             extra = (
#                 "\nPress SPACE to continue"
#                 "\nPress R to repeat BOTH phases"
#             )

#         self.label.config(
#             text=f"Phase {self.current_phase + 1}\n"
#                  f"Explore...{extra}"
#         )

#         self.current_phase += 1

#     # ======================================
#     # Repeat Trial
#     # ======================================
#     def repeat_trial(self):

#         self.current_phase = 0

#         self.label.config(
#             text="Repeating both phases...\nStarting Phase 1"
#         )

#         self.run_phase()

#     # ======================================
#     # Next Phase
#     # ======================================
#     def next_phase(self):

#         self.run_phase()

#     # ======================================
#     # Ask Rating
#     # ======================================
#     def ask_response(self):

#         self.stage = "response"

#         self.label.config(
#             text=
#             "Rate Dissimilarity (0–10)\n\n"
#             "0 = Same\n"
#             "10 = Completely different\n\n"
#             "Press number keys"
#         )

#     # ======================================
#     # Save CSV
#     # ======================================
#     def save_trial(self):

#         file_exists = os.path.isfile(
#             self.filename
#         )

#         with open(
#             self.filename,
#             mode='a',
#             newline=''
#         ) as file:

#             writer = csv.writer(file)

#             if not file_exists:

#                 writer.writerow([
#                     "Timestamp",
#                     "User",
#                     "Trial",
#                     "TotalLoss",
#                     "Phase1",
#                     "Phase2",
#                     "AsymPair",
#                     "Rating"
#                 ])

#             writer.writerow([
#                 datetime.now(),
#                 self.username,
#                 self.trial,
#                 TOTAL_LOSS,
#                 self.phases[0]["loss"],
#                 self.phases[1]["loss"],
#                 self.asym_pair,
#                 self.user_rating
#             ])

#     # ======================================
#     # Run GUI
#     # ======================================
#     def run(self):

#         self.root.mainloop()


# if __name__ == "__main__":

#     Experiment().run()

#############################################################3
#pACKET LOSS 20%

#!/usr/bin/env python3


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

# ==========================================
# TOTAL PACKET LOSS
# ==========================================
TOTAL_LOSS = 20

# ==========================================
# FIXED ASYMMETRIC COMBINATIONS
# ==========================================



ASYM_PAIRS = [
    (1,19),
    (19,1),
    (2,18),
    (18,2),
    (4,16),
    (16,4)
]

# ==========================================
# SYMMETRIC CONDITION
# ==========================================
SYMMETRIC = (10, 10)


class Experiment:

    def __init__(self):

        rospy.init_node('master_packetloss_experiment')

        self.pub = rospy.Publisher(
            '/loss_cmd',
            Int32MultiArray,
            queue_size=10
        )

        # ----------------------------------
        # Camera
        # ----------------------------------
        self.bridge = CvBridge()

        rospy.Subscriber(
            '/camera_feed',
            Image,
            self.image_callback
        )

        # ----------------------------------
        # GUI
        # ----------------------------------
        self.root = tk.Tk()

        self.root.title("Packet Loss Rating Experiment")

        self.root.geometry("700x450+1100+0")

        self.stage = "name"

        self.trial = 0

        # ----------------------------------
        # Trial list
        # ----------------------------------
        self.trials = []

        # Symmetric vs symmetric trial
        self.trials.append({
            "type": "sym_sym",
            "pair": None
        })

        # Symmetric vs asymmetric trials
        for pair in ASYM_PAIRS:
            self.trials.append({
                "type": "asym",
                "pair": pair
            })

        random.shuffle(self.trials)

        self.order_toggle = True

        # ----------------------------------
        # GUI Label
        # ----------------------------------
        self.label = tk.Label(
            self.root,
            text="Enter Name and Press ENTER",
            font=("Arial", 22),
            wraplength=750,
            justify="center"
        )

        self.label.pack(expand=True)

        # ----------------------------------
        # Entry
        # ----------------------------------
        self.entry = tk.Entry(
            self.root,
            font=("Arial", 18)
        )

        self.entry.pack()

        self.entry.focus()

        # ----------------------------------
        # Keyboard Bindings
        # ----------------------------------
        self.root.bind("<Return>", self.handle_enter)

        self.root.bind("<Key>", self.key_handler)

    # ======================================
    # Camera Callback
    # ======================================
    def image_callback(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )

        cv_image = cv2.resize(cv_image, (1000, 700))

        cv2.imshow("Camera Feed", cv_image)

        cv2.resizeWindow("Camera Feed", 1100, 700)

        cv2.moveWindow("Camera Feed", 0, 0)

        cv2.waitKey(1)

    # ======================================
    # Name Entry
    # ======================================
    def handle_enter(self, event):

        if self.stage == "name":

            self.username = self.entry.get().strip()

            if not self.username:
                return

            timestamp = datetime.now().strftime(
                "%Y-%m-%d_%H-%M-%S"
            )

            base_path = "/home/autonomous-lab/Desktop/delay/Packetloss"

            os.makedirs(base_path, exist_ok=True)

            self.filename = os.path.join(
                base_path,
                f"{self.username}_{timestamp}.csv"
            )

            self.entry.pack_forget()

            self.stage = "ready"

            self.label.config(
                text="Press S to Start Trial"
            )

    # ======================================
    # Keyboard Handler
    # ======================================
    def key_handler(self, event):

        # ----------------------------------
        # Start Trial
        # ----------------------------------
        if self.stage == "ready":

            if event.char.lower() == 's':
                self.start_trial()

        # ----------------------------------
        # During Phase
        # ----------------------------------
        elif self.stage == "phase":

            # SPACE → next phase
            if event.keysym == 'space':
                self.next_phase()

            # R → repeat both phases
            elif (
                event.char.lower() == 'r'
                and self.current_phase == 2
            ):
                self.repeat_trial()

        # ----------------------------------
        # Rating Input
        # ----------------------------------
        elif self.stage == "response":

            if event.char.isdigit():

                rating = int(event.char)

                # Handle 10
                if rating == 1 and self.pending_one:

                    rating = 10

                    self.pending_one = False

                elif rating == 1:

                    self.pending_one = True
                    return

                else:
                    self.pending_one = False

                if 0 <= rating <= 10:

                    self.user_rating = rating

                    self.save_trial()

                    self.stage = "ready"

                    self.label.config(
                        text="Saved.\nPress S for next trial"
                    )

    # ======================================
    # Apply Packet Loss
    # ======================================
    def apply_loss(self, master_loss, slave_loss):

        # Clear previous packet loss
        subprocess.call(
            f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
            shell=True
        )

        # Apply MASTER packet loss
        subprocess.call(
            f"sudo tc qdisc add dev {INTERFACE} "
            f"root netem loss {master_loss}%",
            shell=True
        )

        # Send SLAVE packet loss
        msg = Int32MultiArray()

        msg.data = [
            int(master_loss),
            int(slave_loss)
        ]

        self.pub.publish(msg)

    # ======================================
    # Start Trial
    # ======================================
    def start_trial(self):

        if len(self.trials) == 0:

            self.label.config(
                text="All trials completed!"
            )

            return

        self.trial += 1

        trial_data = self.trials.pop()

        # ----------------------------------
        # Symmetric vs Symmetric
        # ----------------------------------
        if trial_data["type"] == "sym_sym":

            self.asym_pair = SYMMETRIC

            self.phases = [
                {
                    "type": "sym",
                    "loss": SYMMETRIC
                },
                {
                    "type": "sym",
                    "loss": SYMMETRIC
                }
            ]

        # ----------------------------------
        # Symmetric vs Asymmetric
        # ----------------------------------
        else:

            self.asym_pair = trial_data["pair"]

            if self.order_toggle:

                self.phases = [
                    {
                        "type": "sym",
                        "loss": SYMMETRIC
                    },
                    {
                        "type": "asym",
                        "loss": self.asym_pair
                    }
                ]

            else:

                self.phases = [
                    {
                        "type": "asym",
                        "loss": self.asym_pair
                    },
                    {
                        "type": "sym",
                        "loss": SYMMETRIC
                    }
                ]

            self.order_toggle = not self.order_toggle

        random.shuffle(self.phases)

        self.current_phase = 0

        self.pending_one = False

        self.run_phase()

    # ======================================
    # Run Phase
    # ======================================
    def run_phase(self):

        if self.current_phase >= 2:

            self.ask_response()

            return

        self.stage = "phase"

        phase = self.phases[self.current_phase]

        master_loss, slave_loss = phase["loss"]

        self.apply_loss(
            master_loss,
            slave_loss
        )

        # UI Text
        if self.current_phase == 0:

            extra = "\nPress SPACE to continue"

        else:

            extra = (
                "\nPress SPACE to continue"
                "\nPress R to repeat BOTH phases"
            )

        self.label.config(
            text=f"Phase {self.current_phase + 1}\n"
                 f"Explore...{extra}"
        )

        self.current_phase += 1

    # ======================================
    # Repeat Trial
    # ======================================
    def repeat_trial(self):

        self.current_phase = 0

        self.label.config(
            text="Repeating both phases...\nStarting Phase 1"
        )

        self.run_phase()

    # ======================================
    # Next Phase
    # ======================================
    def next_phase(self):

        self.run_phase()

    # ======================================
    # Ask Rating
    # ======================================
    def ask_response(self):

        self.stage = "response"

        self.label.config(
            text=
            "Rate Dissimilarity (0–10)\n\n"
            "0 = Same\n"
            "10 = Completely different\n\n"
            "Press number keys"
        )

    # ======================================
    # Save CSV
    # ======================================
    def save_trial(self):

        file_exists = os.path.isfile(
            self.filename
        )

        with open(
            self.filename,
            mode='a',
            newline=''
        ) as file:

            writer = csv.writer(file)

            if not file_exists:

                writer.writerow([
                    "Timestamp",
                    "User",
                    "Trial",
                    "TotalLoss",
                    "Phase1",
                    "Phase2",
                    "AsymPair",
                    "Rating"
                ])

            writer.writerow([
                datetime.now(),
                self.username,
                self.trial,
                TOTAL_LOSS,
                self.phases[0]["loss"],
                self.phases[1]["loss"],
                self.asym_pair,
                self.user_rating
            ])

    # ======================================
    # Run GUI
    # ======================================
    def run(self):

        self.root.mainloop()


if __name__ == "__main__":

    Experiment().run()
#########################################################################3
#Asym+0



# import rospy
# from std_msgs.msg import Int32MultiArray
# import tkinter as tk
# import random
# import subprocess
# import csv
# import os
# from datetime import datetime

# INTERFACE = "enp2s0"
# TOTAL_DELAY = 12   # Change as needed


# class Experiment:

#     def __init__(self):

#         rospy.init_node('master_delay_node')
#         self.pub = rospy.Publisher('/delay_cmd', Int32MultiArray, queue_size=10)

#         self.root = tk.Tk()
#         self.root.title("Delay Perception Experiment")
#         self.root.geometry("1000x650")

#         self.stage = "name"
#         self.trial = 0

#         # Generate asymmetric pairs
#         self.asym_pairs = self.generate_all_pairs()
#         random.shuffle(self.asym_pairs)

#         self.label = tk.Label(self.root, text="Enter Name and Press ENTER",
#                               font=("Arial", 24))
#         self.label.pack(expand=True)

#         self.entry = tk.Entry(self.root, font=("Arial", 20))
#         self.entry.pack()
#         self.entry.focus()

#         self.root.bind("<Return>", self.handle_enter)
#         self.root.bind("<Key>", self.key_handler)

#     # ----------------------------------------
#     def generate_all_pairs(self):
#         pairs = []
#         for i in range(1, TOTAL_DELAY):
#             if i != TOTAL_DELAY - i:
#                 pairs.append((i, TOTAL_DELAY - i))
#         return pairs

#     # ----------------------------------------
#     def handle_enter(self, event):
#         if self.stage == "name":
#             self.username = self.entry.get().strip()
#             if not self.username:
#                 return

#             timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

#             base_path = "/home/autonomous-lab/Desktop/delay"
#             os.makedirs(base_path, exist_ok=True)

#             self.filename = os.path.join(base_path,
#                                         f"{self.username}_{timestamp}.csv")

#             self.entry.pack_forget()
#             self.stage = "ready"
#             self.label.config(text="Press S to Start Trial")

#     # ----------------------------------------
#     def key_handler(self, event):

#         # Start trial
#         if self.stage == "ready" and event.char.lower() == 's':
#             self.start_trial()

#         # Move to next phase ONLY manually
#         elif self.stage == "phase" and event.char.lower() == 'n':
#             self.next_phase()

#         # Answer
#         elif self.stage == "response" and event.char in ['1', '2', '3']:
#             self.user_answer = int(event.char)
#             self.save_trial()
#             self.stage = "ready"
#             self.label.config(text="Saved.\nPress S for next trial")

#     # ----------------------------------------
#     def apply_delay(self, m, s):

#         subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
#                         shell=True)

#         subprocess.call(
#             f"sudo tc qdisc add dev {INTERFACE} root netem delay {m}ms",
#             shell=True)

#         msg = Int32MultiArray()
#         msg.data = [m, s]
#         self.pub.publish(msg)

#     # ----------------------------------------
#     def start_trial(self):

#         if len(self.asym_pairs) == 0:
#             self.label.config(text="All asymmetric pairs used!")
#             return

#         self.trial += 1

#         # Pick unique asymmetric pair
#         self.asym_m, self.asym_s = self.asym_pairs.pop()

#         zero = (0, 0)

#         # Build phases
#         self.phases = [
#             {"type": "zero", "delay": zero},
#             {"type": "zero", "delay": zero},
#             {"type": "asym", "delay": (self.asym_m, self.asym_s)}
#         ]

#         random.shuffle(self.phases)

#         for i, p in enumerate(self.phases):
#             if p["type"] == "asym":
#                 self.correct_phase = i + 1

#         self.current_phase = 0
#         self.run_phase()

#     # ----------------------------------------
#     def run_phase(self):

#         if self.current_phase >= 3:
#             self.ask_response()
#             return

#         self.stage = "phase"

#         phase = self.phases[self.current_phase]
#         m, s = phase["delay"]

#         self.apply_delay(m, s)

#         self.label.config(
#             text=f"Phase {self.current_phase + 1}\nExplore...\nPress N to go next"
#         )

#         self.current_phase += 1

#     # ----------------------------------------
#     def next_phase(self):
#         self.run_phase()

#     # ----------------------------------------
#     def ask_response(self):

#         self.stage = "response"

#         self.label.config(
#             text="Which phase is DIFFERENT?\n\nPress 1 / 2 / 3"
#         )

#     # ----------------------------------------
#     def save_trial(self):

#         file_exists = os.path.isfile(self.filename)

#         with open(self.filename, mode='a', newline='') as file:
#             writer = csv.writer(file)

#             if not file_exists:
#                 writer.writerow([
#                     "Timestamp", "User", "Trial",
#                     "TotalDelay",
#                     "Phase1", "Phase2", "Phase3",
#                     "Asym_Master", "Asym_Slave",
#                     "CorrectPhase", "UserAnswer"
#                 ])

#             writer.writerow([
#                 datetime.now(),
#                 self.username,
#                 self.trial,
#                 TOTAL_DELAY,
#                 self.phases[0]["delay"],
#                 self.phases[1]["delay"],
#                 self.phases[2]["delay"],
#                 self.asym_m,
#                 self.asym_s,
#                 self.correct_phase,
#                 self.user_answer
#             ])

#     # ----------------------------------------
#     def run(self):
#         self.root.mainloop()


# if __name__ == "__main__":
#     Experiment().run()



#########################################################################################

# #SYM+0


# import rospy
# from std_msgs.msg import Int32MultiArray
# import tkinter as tk
# import random
# import subprocess
# import csv
# import os
# from datetime import datetime

# INTERFACE = "enp2s0"

# TOTAL_DELAY = 16


# class Experiment:

#     def __init__(self):

#         rospy.init_node('master_delay_node')
#         self.pub = rospy.Publisher('/delay_cmd', Int32MultiArray, queue_size=10)

#         self.root = tk.Tk()
#         self.root.title("Delay Perception Experiment")
#         self.root.geometry("1000x650")

#         self.stage = "name"
#         self.trial = 0

#         self.label = tk.Label(self.root, text="Enter Name and Press ENTER",
#                               font=("Arial", 24))
#         self.label.pack(expand=True)

#         self.entry = tk.Entry(self.root, font=("Arial", 20))
#         self.entry.pack()
#         self.entry.focus()

#         self.root.bind("<Return>", self.handle_enter)
#         self.root.bind("<Key>", self.key_handler)

#     # ----------------------------------------
#     def handle_enter(self, event):
#         if self.stage == "name":
#             self.username = self.entry.get().strip()
#             if not self.username:
#                 return

#             timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

#             base_path = "/home/autonomous-lab/Desktop/delay"
#             os.makedirs(base_path, exist_ok=True)

#             self.filename = os.path.join(base_path,
#                                         f"{self.username}_{timestamp}.csv")

#             self.entry.pack_forget()
#             self.stage = "ready"
#             self.label.config(text="Press S to Start Trial")

#     # ----------------------------------------
#     def key_handler(self, event):

#         if self.stage == "ready" and event.char.lower() == 's':
#             self.start_trial()

#         elif self.stage == "phase" and event.char.lower() == 'n':
#             self.next_phase()

#         elif self.stage == "response" and event.char in ['1', '2', '3']:
#             self.user_answer = int(event.char)
#             self.save_trial()
#             self.stage = "ready"
#             self.label.config(text="Saved.\nPress S for next trial")

#     # ----------------------------------------
#     def apply_delay(self, m, s):

#         subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null",
#                         shell=True)

#         subprocess.call(
#             f"sudo tc qdisc add dev {INTERFACE} root netem delay {m}ms",
#             shell=True)

#         msg = Int32MultiArray()
#         msg.data = [m, s]
#         self.pub.publish(msg)

#     # ----------------------------------------
#     def start_trial(self):

#         self.trial += 1

#         # Symmetric delay
#         sym_m = TOTAL_DELAY // 2
#         sym_s = TOTAL_DELAY // 2

#         zero = (0, 0)

#         # Build phases
#         self.phases = [
#             {"type": "zero", "delay": zero},
#             {"type": "zero", "delay": zero},
#             {"type": "sym", "delay": (sym_m, sym_s)}
#         ]

#         random.shuffle(self.phases)

#         # Find correct phase
#         for i, p in enumerate(self.phases):
#             if p["type"] == "sym":
#                 self.correct_phase = i + 1

#         self.current_phase = 0
#         self.run_phase()

#     # ----------------------------------------
#     def run_phase(self):

#         if self.current_phase >= 3:
#             self.ask_response()
#             return

#         self.stage = "phase"

#         phase = self.phases[self.current_phase]
#         m, s = phase["delay"]

#         self.apply_delay(m, s)

#         self.label.config(
#             text=f"Phase {self.current_phase + 1}\nExplore...\nPress N to go next"
#         )

#         self.current_phase += 1

#     # ----------------------------------------
#     def next_phase(self):
#         self.run_phase()

#     # ----------------------------------------
#     def ask_response(self):

#         self.stage = "response"

#         self.label.config(
#             text="Which phase is DIFFERENT?\n\nPress 1 / 2 / 3"
#         )

#     # ----------------------------------------
#     def save_trial(self):

#         file_exists = os.path.isfile(self.filename)

#         with open(self.filename, mode='a', newline='') as file:
#             writer = csv.writer(file)

#             if not file_exists:
#                 writer.writerow([
#                     "Timestamp", "User", "Trial",
#                     "TotalDelay",
#                     "Phase1", "Phase2", "Phase3",
#                     "Sym_Master", "Sym_Slave",
#                     "CorrectPhase", "UserAnswer"
#                 ])

#             writer.writerow([
#                 datetime.now(),
#                 self.username,
#                 self.trial,
#                 TOTAL_DELAY,
#                 self.phases[0]["delay"],
#                 self.phases[1]["delay"],
#                 self.phases[2]["delay"],
#                 TOTAL_DELAY // 2,
#                 TOTAL_DELAY // 2,
#                 self.correct_phase,
#                 self.user_answer
#             ])

#     # ----------------------------------------
#     def run(self):
#         self.root.mainloop()


# if __name__ == "__main__":
#     Experiment().run()