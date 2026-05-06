#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.fftpack import fft

# Step 1: Load all CSV files into a dictionary
data_dict = {}
file_names = ['/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_circle_v1_k0.5.csv',
              '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_circle_v1_k8.5.csv', 
              '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_circle_v1_k15.csv',
              '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_circle_v1.5_k0.5.csv', 
              '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_circle_v1.5_k8.5.csv', 
              '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_circle_v1.5_k15.csv',
              '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_circle_v2_k0.5.csv', 
              '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_circle_v2_k8.5.csv', 
              '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_circle_v2_k15.csv']

# file_names = ['/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_triangle_v1_k0.5.csv',
#               '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_triangle_v1_k8.5.csv', 
#               '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_triangle_v1_k15.csv',
#               '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_triangle_v1.5_k0.5.csv', 
#               '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_triangle_v1.5_k8.5.csv', 
#               '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_triangle_v1.5_k15.csv',
#               '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_triangle_v2_k0.5.csv', 
#               '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_triangle_v2_k8.5.csv', 
#               '/home/user/experiment_data/06_sept_2024/Accelerometer Files/ElecLab_iitjwlan_triangle_v2_k15.csv']

for file in file_names:
    data = pd.read_csv(file, header=None, names=['timestamp', 'x', 'y', 'z'])
    data['time'] = (data['timestamp'] - data['timestamp'][0]) / 1000.0  # Convert to seconds
    
    # Convert columns to numeric, coercing errors to NaN
    data[['x', 'y', 'z']] = data[['x', 'y', 'z']].apply(pd.to_numeric, errors='coerce')
    
    # Drop rows with NaN values after conversion
    data = data.dropna()
    
    data_dict[file] = data

# Step 2: Time-Domain Plots for Comparison
plt.figure(figsize=(15, 10))
for i, (file, data) in enumerate(data_dict.items(), 1): 
    plt.subplot(3, 3, i)
    
    # Convert pandas Series to numpy array for plotting
    time = data['time'].to_numpy()
    x = data['x'].to_numpy()
    y = data['y'].to_numpy()
    z = data['z'].to_numpy()

    plt.plot(time, x, label='X-Axis')
    plt.plot(time, y, label='Y-Axis')
    plt.plot(time, z, label='Z-Axis')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration')
    plt.title(f'Experiment {i}')
    plt.legend()

plt.tight_layout()
plt.show()

# Step 3: Frequency Analysis and Comparison
fs = 1000  # Sampling frequency (Hz)

plt.figure(figsize=(15, 10))
for i, (file, data) in enumerate(data_dict.items(), 1):
    N = len(data['x'])
    
    # Convert to numpy arrays for FFT calculations
    x_data = data['x'].to_numpy()
    X = fft(x_data - np.mean(x_data))
    frequencies = np.fft.fftfreq(N, d=1/fs)
    
    plt.subplot(3, 3, i)
    plt.plot(frequencies[:N // 2], np.abs(X)[:N // 2], label='X-Axis FFT')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Amplitude')
    plt.title(f'Frequency Spectrum - Experiment {i}')

plt.tight_layout()
plt.show()

# Step 4: Statistical Analysis and Summary Plot
rms_values = {}
for file, data in data_dict.items():
    # Calculate RMS values for each axis
    rms_x = np.sqrt(np.mean(data['x'].to_numpy() ** 2))
    rms_y = np.sqrt(np.mean(data['y'].to_numpy() ** 2))
    rms_z = np.sqrt(np.mean(data['z'].to_numpy() ** 2))
    rms_values[file] = [rms_x, rms_y, rms_z]

# Create a summary plot for RMS values
rms_df = pd.DataFrame(rms_values, index=['RMS_X', 'RMS_Y', 'RMS_Z']).T
rms_df.plot(kind='bar', figsize=(10, 6))
plt.title('RMS Values for Each Experiment')
plt.ylabel('RMS Acceleration')
plt.show()