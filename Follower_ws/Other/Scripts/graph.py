#!/usr/bin/env python3

# import pandas as pd
# import matplotlib.pyplot as plt


# #csv_file = '/home/user/Desktop/delay/Others1//forcez/force.csv'
# csv_file = '/home/user/Desktop/delay/robotforce/force.csv'



# try:
#     data = pd.read_csv(csv_file, header=None)
# except pd.errors.ParserError:
#     data = pd.read_csv(csv_file)


# timestamps = data.iloc[:, 1]
# force = data.iloc[:, 4]

# timestamps = pd.to_numeric(timestamps, errors='coerce')
# force = pd.to_numeric(force, errors='coerce')

# valid = ~timestamps.isna() & ~force.isna()
# timestamps = timestamps[valid].to_numpy()
# force = force[valid].to_numpy()

# #Plot 
# plt.figure(figsize=(10, 5))
# plt.plot(timestamps, force, label='Force', linewidth=1.5)

# plt.title('Force vs Time', fontsize=14)
# plt.xlabel('Times(s)', fontsize=12)
# plt.ylabel('Force (N)', fontsize=12)
# plt.grid(True)
# plt.legend()
# plt.tight_layout()
# plt.show()
# import csv
# import matplotlib.pyplot as plt

# csv_file = r"/home/user/Desktop/delay/250/force_rtd.csv"

# delays_ms = []

# # Read the CSV
# with open(csv_file, "r") as file:
#     reader = csv.reader(file)
#     next(reader)  # skip header row

#     for row in reader:
#         if row[1].strip() != "":
#             delay_seconds = float(row[1])
#             delays_ms.append(delay_seconds * 1000)  # convert to ms

# # Plot only histogram
# plt.figure(figsize=(8, 5))
# plt.hist(delays_ms, bins=100)  # you can change bins (20,40,etc)
# plt.title("Communication Delay Histogram")
# plt.xlabel("Delay (ms)")
# plt.ylabel("Frequency")
# plt.grid(True)
# plt.show()


# import csv
# import matplotlib.pyplot as plt
# import numpy as np

# csv_file = r"/home/user/Desktop/delay/1000/force_rtd.csv"

# delays_ms = []

# with open(csv_file, "r") as file:
#     reader = csv.reader(file)
#     next(reader)

#     for row in reader:
#         if row[1].strip() != "":
#             delay_seconds = float(row[1])
#             delays_ms.append(delay_seconds * 1000)

# # Plot histogram
# plt.figure(figsize=(8,5))
# plt.hist(delays_ms, bins=50)

# # Set x-axis ticks every 1 ms
# min_val = int(min(delays_ms))
# max_val = int(max(delays_ms))
# plt.xticks(np.arange(min_val, max_val + 1, 1))   # step = 1 ms

# plt.title("Communication Delay Histogram")
# plt.xlabel("Delay (ms)")
# plt.ylabel("Frequency")
# plt.grid(True)
# plt.show()

# import pandas as pd
# import matplotlib.pyplot as plt

# # Load CSV file
# file_path = '/home/user/Desktop/delay/force.csv'   
# data = pd.read_csv(file_path)

# # Zero Based Indexing
# y = data.iloc[:, 7]

# # Plot
# plt.figure()
# plt.plot(y)
# plt.xlabel('Sample Index')
# plt.ylabel('Column 3 Values')
# plt.title('Plot')
# plt.grid(True)
# plt.show()



# import pandas as pd
# import matplotlib.pyplot as plt

# file_path = '/home/user/Desktop/delay/force.csv'
# data = pd.read_csv(file_path)

# # Extract columns
# time = data.iloc[:, 1].astype(float).to_numpy()
# force = data.iloc[:, 7].astype(float).to_numpy()

# # Optional: remove NaNs safely
# mask = ~pd.isna(time) & ~pd.isna(force)
# time = time[mask]
# force = force[mask]

# # Plot
# plt.figure()
# plt.plot(time, force)
# plt.xlabel('Time')
# plt.ylabel('Force')
# plt.title('Force vs Time')
# plt.grid(True)
# plt.show()
#############################################################
# import pandas as pd
# import matplotlib.pyplot as plt


# force_file = '/home/user/Desktop/delay/force.csv'
# force_data = pd.read_csv(force_file)

# force_time = force_data.iloc[:, 1].astype(float).to_numpy()
# force = force_data.iloc[:, 5].astype(float).to_numpy()

# mask_f = ~pd.isna(force_time) & ~pd.isna(force)
# force_time = force_time[mask_f]
# force = force[mask_f]


# pose_file = '/home/user/Desktop/delay/pose.csv'   
# pose_data = pd.read_csv(pose_file)

# pose_time = pose_data.iloc[:, 1].astype(float).to_numpy()   
# pose = pose_data.iloc[:, 3].astype(float).to_numpy()        

# mask_p = ~pd.isna(pose_time) & ~pd.isna(pose)
# pose_time = pose_time[mask_p]
# pose = pose[mask_p]


# plt.figure(figsize=(10, 6))

# # Subplot 1: Force vs Time
# plt.subplot(2, 1, 1)
# plt.plot(force_time, force)
# plt.xlabel('Time')
# plt.ylabel('Force')
# plt.title('Force vs Time')
# plt.grid(True)

# # Subplot 2: Pose vs Time
# plt.subplot(2, 1, 2)
# plt.plot(pose_time, pose)
# plt.xlabel('Time')
# plt.ylabel('Pose')
# plt.title('Pose vs Time')
# plt.grid(True)

# plt.tight_layout()
# plt.show()

######################


# import numpy as np
# import matplotlib.pyplot as plt

# # Read CSV data (skip header row)
# data = np.loadtxt('/home/user/Desktop/delay/virtual_plane_interaction.csv', 
#                   delimiter=',', skiprows=1)

# # Extract columns
# time = data[:, 0]           # Column 0: Time
# position_z = data[:, 3]     # Column 3: Position Z
# force_z = data[:, 6]        # Column 6: Force Z

# # Make time relative to start
# time_relative = time - time[0]

# # Create plots
# fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

# # Z Position plot
# ax1.plot(time_relative, position_z, 'b-', linewidth=1.5)
# ax1.axhline(y=0.1628, color='r', linestyle='--', linewidth=2, label='Plane Height')
# ax1.set_xlabel('Time (s)')
# ax1.set_ylabel('Z Position (m)')
# ax1.set_title('Z Position vs Time')
# ax1.grid(True, alpha=0.3)
# ax1.legend()

# # Z Force plot
# ax2.plot(time_relative, force_z, 'r-', linewidth=1.5)
# ax2.axhline(y=2.0, color='g', linestyle='--', linewidth=2, label='Max Force')
# ax2.set_xlabel('Time (s)')
# ax2.set_ylabel('Force Z (N)')
# ax2.set_title('Force Z vs Time')
# ax2.grid(True, alpha=0.3)
# ax2.legend()

# plt.tight_layout()
# plt.savefig('/home/user/Desktop/delay/plot.png', dpi=300)
# plt.show()


######################


# import pandas as pd
# import matplotlib.pyplot as plt


# force_file = '/home/user/Desktop/delay/force.csv'
# force_data = pd.read_csv(force_file)

# force_time = force_data.iloc[:, 1].astype(float).to_numpy()
# forcex = force_data.iloc[:, 5].astype(float).to_numpy()
# forcey = force_data.iloc[:, 6].astype(float).to_numpy()

# mask_f = ~pd.isna(force_time) & ~pd.isna(forcex) & ~pd.isna(forcey)
# force_time = force_time[mask_f]
# forcex = forcex[mask_f]


# pose_file = '/home/user/Desktop/delay/pose.csv'   
# pose_data = pd.read_csv(pose_file)

# pose_time = pose_data.iloc[:, 1].astype(float).to_numpy()   
# pose = pose_data.iloc[:, 3].astype(float).to_numpy()        

# mask_p = ~pd.isna(pose_time) & ~pd.isna(pose)
# pose_time = pose_time[mask_p]
# pose = pose[mask_p]


# plt.figure(figsize=(10, 6))

# # Subplot 1: Force vs Time
# plt.subplot(2, 1, 1)
# plt.plot(force_time, force)
# plt.xlabel('Time')
# plt.ylabel('Force')
# plt.title('Force vs Time')
# plt.grid(True)

# # Subplot 2: Pose vs Time
# plt.subplot(2, 1, 2)
# plt.plot(pose_time, pose)
# plt.xlabel('Time')
# plt.ylabel('Pose')
# plt.title('Pose vs Time')
# plt.grid(True)

# plt.tight_layout()
# plt.show()




# import pandas as pd
# import matplotlib.pyplot as plt
# from matplotlib.ticker import FormatStrFormatter

# # Load CSV
# df = pd.read_csv('/home/user/Desktop/delay/pose.csv')

# # Extract columns
# timestamps = pd.to_numeric(df.iloc[:, 0], errors='coerce')
# delays = pd.to_numeric(df.iloc[:, 1], errors='coerce') * 1000  # sec → ms

# # Remove NaNs
# mask = (~timestamps.isna()) & (~delays.isna())
# timestamps = timestamps[mask]
# delays = delays[mask]

# # Sort by timestamp (important for time plots)
# order = timestamps.argsort()
# timestamps = timestamps.iloc[order].values
# delays = delays.iloc[order].values

# # Average delay
# avg_delay_ms = delays.mean()
# print(f'Average Pose Delay: {avg_delay_ms:.5f} milliseconds')

# # Plot
# plt.figure(figsize=(10, 4))
# plt.plot(timestamps, delays, 'b.-', label='Pose Delay (ms)')
# plt.axhline(
#     y=avg_delay_ms,
#     color='r',
#     linestyle='--',
#     linewidth=1.5,
#     label=f'Avg Delay = {avg_delay_ms:.2f} ms'
# )

# plt.xlabel('Timestamp (s)', fontsize=12)
# plt.ylabel('Delay (ms)', fontsize=12)
# plt.grid(True)
# plt.gca().xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
# plt.legend()
# plt.tight_layout()
# plt.show()




import pandas as pd
import matplotlib.pyplot as plt


df = pd.read_csv('/home/user/Desktop/delay/pose_waiting_time.csv')


timestamps = df.iloc[:, 0].values
delays = df.iloc[:, 3].values * 1000  # Convert delays to ms


avg_delay_ms = delays.mean()
# max_delay_ms = delays.max()
# print(f'Maximum Pose Delay: {max_delay_ms:.5f} milliseconds')

print(f'Average Pose Delay: {avg_delay_ms:.5f} milliseconds')


# Plot
plt.figure()
plt.plot(timestamps, delays, 'b.-', label='Pose Delay (ms)')
plt.axhline(y=avg_delay_ms, color='r', linestyle='--',
         linewidth=1.5, label=f'Avg Delay = {avg_delay_ms:.2f} ms')


plt.xlabel('Timestamp (s)', fontsize=12)
plt.ylabel('Delay (ms)', fontsize=12)
plt.grid(True)
plt.ticklabel_format(axis='x', style='plain')
plt.gca().xaxis.set_major_formatter(plt.matplotlib.ticker.FormatStrFormatter('%.2f'))
plt.legend()

