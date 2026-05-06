#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Step 1: Read the CSV file
df = pd.read_csv('/home/user/experiment_data/18_Aug_2024/exp_c_5_f/robot_pose_data.csv')  # Replace 'data.csv' with your actual file path

# Step 2: Extract signals and timestamps
time = df['Time'].values  # Time column
master_signal = df['Stylus Pose X'].values  # Master signal column
slave_signal = df['EE Pose X'].values  # Slave signal column

# # Step 3: Parameters for the sliding window with non-uniform sampling
# window_duration = 0.01  # Duration of each window in seconds (adjust as needed) 2.0
# step_duration = 0.001  # Step duration for sliding window in seconds 0.5

# # Step 4: Initialize lists to store results
# time_intervals = []  # To store the middle time points of each window
# delays = []  # To store calculated delays

# # Step 5: Sliding window approach with time-based windows
# start_index = 0

# while start_index < len(time):
#     # Find the end index of the current window based on window_duration
#     end_index = start_index
#     while end_index < len(time) and (time[end_index] - time[start_index]) <= window_duration:
#         end_index += 1
    
#     if end_index >= len(time):  # Exit if we've reached the end of the data
#         break

#     # Extract windowed segments
#     master_window = master_signal[start_index:end_index]
#     slave_window = slave_signal[start_index:end_index]
#     time_window = time[start_index:end_index]
    
#     # Calculate cross-correlation
#     correlation = np.correlate(slave_window - np.mean(slave_window), master_window - np.mean(master_window), mode='full')
#     lags = np.arange(-len(master_window) + 1, len(master_window))
    
#     # Convert lags from samples to time using actual timestamps
#     lag_times = [time_window[0] - time_window[lag] if lag < 0 else time_window[lag] - time_window[0] for lag in lags]
#     delay_time = lag_times[np.argmax(correlation)]  # Delay in seconds

#     # Store results
#     delays.append(delay_time)
#     time_intervals.append(time_window[len(time_window) // 2])  # Middle of the current window

#     # Move to the next window
#     next_start_time = time[start_index] + step_duration
#     start_index = np.searchsorted(time, next_start_time)

# # Step 6: Plot time-varying delay
# plt.figure(figsize=(10, 4))
# plt.plot(time_intervals, delays)
# plt.title('Time-Varying Delay between Master and Slave Signals (Non-Uniform Sampling)')
# plt.xlabel('Time (seconds)')
# plt.ylabel('Delay (seconds)')
# plt.grid()
# plt.show()





from scipy.signal import stft

# Step 3: Compute STFT for both signals
# Define parameters for STFT
fs = 1 / np.mean(np.diff(time))  # Sampling frequency estimated from time data
nperseg = 256  # Number of samples per segment (adjust as needed)

# Compute STFT for both signals
f_master, t_master, Zxx_master = stft(master_signal, fs, nperseg=nperseg)
f_slave, t_slave, Zxx_slave = stft(slave_signal, fs, nperseg=nperseg)

# Step 4: Calculate phase difference
phase_diff = np.angle(Zxx_slave) - np.angle(Zxx_master)

# Step 5: Convert phase difference to time delay (tau = phase_diff / omega)
omega = 2 * np.pi * f_master[:, np.newaxis]  # Angular frequency (rad/s)
time_delay = phase_diff / omega  # Delay in seconds

# Handle cases where omega is zero to avoid division by zero
time_delay[np.isinf(time_delay) | np.isnan(time_delay)] = 0

# Step 6: Average time delay across relevant frequencies to get overall delay estimate
# Select a frequency range to consider for averaging (e.g., ignore very low/high frequencies)
min_freq = 200  # Minimum frequency in Hz (adjust as needed)
max_freq = 500  # Maximum frequency in Hz (adjust as needed)
freq_indices = (f_master >= min_freq) & (f_master <= max_freq)

# Average delay across selected frequencies
average_delay = np.mean(time_delay[freq_indices, :], axis=0)

# Step 7: Plot the time-varying delay
plt.figure(figsize=(10, 4))
plt.plot(t_master, average_delay, linestyle='-', color='b', label='Estimated Delay')
plt.axhline(0, color='black', linestyle='--', linewidth=0.8)  # Zero delay line
plt.title('Time-Varying Delay between Master and Slave Signals using STFT')
plt.xlabel('Time (seconds)')
plt.ylabel('Delay (seconds)')
plt.grid()
plt.legend()
plt.show()