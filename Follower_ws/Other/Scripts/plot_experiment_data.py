#!/usr/bin/env python3

# import pandas as pd
# import matplotlib.pyplot as plt

# # List of CSV filenames
# # csv_files = [
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k0.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k8.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k15/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k0.5/pose_data.csv',
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k8.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k15/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k0.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k8.5/pose_data.csv',
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k15/pose_data.csv'
# # ]

# csv_files = [
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k0.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k15/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k0.5/pose_data.csv',
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k15/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k0.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k8.5/pose_data.csv',
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k15/pose_data.csv'
# ]

# # Create a figure and axis for plotting
# plt.figure(figsize=(10, 8))

# # Loop through each CSV file and plot EE Pose X vs. EE Pose Y
# for i, file in enumerate(csv_files, 1):
#     # Read the CSV file
#     df = pd.read_csv(file)
    
#     # Ensure the data is clean and numeric, dropping any NaN values
#     df['EE Pose X'] = pd.to_numeric(df['EE Pose X'], errors='coerce')
#     df['EE Pose Y'] = pd.to_numeric(df['EE Pose Y'], errors='coerce')
#     df = df.dropna(subset=['EE Pose X', 'EE Pose Y'])

#     # Convert the Series to numpy arrays
#     x_values = df['EE Pose X'].to_numpy()
#     y_values = df['EE Pose Y'].to_numpy()
    
#     # Plot EE Pose X vs. EE Pose Y with a dynamic label
#     plt.plot(x_values, y_values, label=f'EE Pose Data Set {i}')

# # Plot Stylus Pose X vs. Stylus Pose Y for the first dataset
# stylus_df = pd.read_csv(csv_files[0])  # You can change the index to select another dataset

# # Ensure Stylus data is clean and numeric, dropping any NaN values
# stylus_df['Stylus Pose X'] = pd.to_numeric(stylus_df['Stylus Pose X'], errors='coerce')
# stylus_df['Stylus Pose Y'] = pd.to_numeric(stylus_df['Stylus Pose Y'], errors='coerce')
# stylus_df = stylus_df.dropna(subset=['Stylus Pose X', 'Stylus Pose Y'])

# # Convert the Series to numpy arrays
# stylus_x = stylus_df['Stylus Pose X'].to_numpy()
# stylus_y = stylus_df['Stylus Pose Y'].to_numpy()

# # Plot Stylus Pose X vs. Stylus Pose Y
# plt.plot(stylus_x, stylus_y, label='Stylus Pose (Data Set 1)', linestyle='--')

# # Set the title and labels for the plot
# plt.title('EE Pose X vs. EE Pose Y and Stylus Pose X vs. Stylus Pose Y')
# plt.xlabel('X Position')
# plt.ylabel('Y Position')

# # Make axes equal
# plt.axis('equal')

# # Turn on grid
# plt.grid(True)

# # Show legend
# plt.legend()

# # Display the plot
# plt.show()

'''for RMSE (in single plot)'''

# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# # List of CSV filenames
# # csv_files = [
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k0.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k8.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k15/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k0.5/pose_data.csv',
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k8.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k15/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k0.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k8.5/pose_data.csv',
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k15/pose_data.csv'
# # ]

# csv_files = [
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k0.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k15/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k0.5/pose_data.csv',
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k15/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k0.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k8.5/pose_data.csv',
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k15/pose_data.csv'
# ]

# # Generate simple labels for the CSV files
# #simple_labels = [f'csv_{i+1}' for i in range(len(csv_files))]
# simple_labels = ['v=1,k=0.5','v=1,k=8.5','v=1,k=15','v=1.5,k=0.5','v=1.5,k=8.5','v=1.5,k=15','v=2,k=0.5','v=2,k=8.5','v=2,k=15']

# # Function to calculate RMSE
# def calculate_rmse(column1, column2):
#     return np.sqrt(((column1 - column2) ** 2).mean())

# # Initialize an empty list to store RMSE results
# rmse_results = []

# # Loop through each CSV file and calculate RMSE
# for file in csv_files:
#     df = pd.read_csv(file)
    
#     # Calculate RMSE for X, Y, Z positions
#     rmse_x = calculate_rmse(df['EE Pose X'], df['Stylus Pose X'])
#     rmse_y = calculate_rmse(df['EE Pose Y'], df['Stylus Pose Y'])
#     rmse_z = calculate_rmse(df['EE Pose Z'], df['Stylus Pose Z'])

#     # Calculate RMSE for Roll, Pitch, Yaw orientations
#     rmse_roll = calculate_rmse(df['EE Pose Roll'], df['Stylus Pose Roll'])
#     rmse_pitch = calculate_rmse(df['EE Pose Pitch'], df['Stylus Pose Pitch'])
#     rmse_yaw = calculate_rmse(df['EE Pose Yaw'], df['Stylus Pose Yaw'])

#     # Calculate overall RMSE
#     overall_rmse = np.sqrt(
#         np.mean(
#             (df['EE Pose X'] - df['Stylus Pose X'])**2 +
#             (df['EE Pose Y'] - df['Stylus Pose Y'])**2 +
#             (df['EE Pose Z'] - df['Stylus Pose Z'])**2 +
#             (df['EE Pose Roll'] - df['Stylus Pose Roll'])**2 +
#             (df['EE Pose Pitch'] - df['Stylus Pose Pitch'])**2 +
#             (df['EE Pose Yaw'] - df['Stylus Pose Yaw'])**2
#         )
#     )
    
#     # Append the results to the list
#     rmse_results.append({
#         'File': file,
#         'RMSE_X': rmse_x,
#         'RMSE_Y': rmse_y,
#         'RMSE_Z': rmse_z,
#         'RMSE_Roll': rmse_roll,
#         'RMSE_Pitch': rmse_pitch,
#         'RMSE_Yaw': rmse_yaw,
#         'Overall_RMSE': overall_rmse
#     })

# # Convert the RMSE results to a DataFrame
# rmse_df = pd.DataFrame(rmse_results)

# # Ensure the 'File' column is treated as strings for plotting
# rmse_df['File'] = simple_labels  # Use simple labels instead of full file paths

# # Convert the 'File' column and all RMSE columns to numpy arrays for plotting
# x_labels = rmse_df['File'].to_numpy()
# rmse_x = rmse_df['RMSE_X'].to_numpy()
# rmse_y = rmse_df['RMSE_Y'].to_numpy()
# rmse_z = rmse_df['RMSE_Z'].to_numpy()
# rmse_roll = rmse_df['RMSE_Roll'].to_numpy()
# rmse_pitch = rmse_df['RMSE_Pitch'].to_numpy()
# rmse_yaw = rmse_df['RMSE_Yaw'].to_numpy()
# overall_rmse = rmse_df['Overall_RMSE'].to_numpy()

# # Plot RMSE for comparison
# plt.figure(figsize=(15, 8))

# # Plot individual RMSE values
# plt.plot(x_labels, rmse_x, marker='o', label='RMSE X')
# plt.plot(x_labels, rmse_y, marker='o', label='RMSE Y')
# plt.plot(x_labels, rmse_z, marker='o', label='RMSE Z')
# plt.plot(x_labels, rmse_roll, marker='o', label='RMSE Roll')
# plt.plot(x_labels, rmse_pitch, marker='o', label='RMSE Pitch')
# plt.plot(x_labels, rmse_yaw, marker='o', label='RMSE Yaw')
# plt.plot(x_labels, overall_rmse, marker='o', label='Overall RMSE', linestyle='--', linewidth=2)

# # Customize plot
# plt.xticks(rotation=45)
# plt.title('RMSE comparison (ElecLab_IITJWlan_Circle)')
# plt.xlabel('Experiments')
# plt.ylabel('RMSE')
# plt.grid(True)
# plt.legend()
# plt.tight_layout()

# # Show plot
# plt.show()

'''for Delay comparison'''

# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.signal import correlate

# # List of CSV filenames
# # csv_files = [
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k0.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k8.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k15/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k0.5/pose_data.csv',
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k8.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k15/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k0.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k8.5/pose_data.csv',
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k15/pose_data.csv'
# # ]
# csv_files = [
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k0.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k15/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k0.5/pose_data.csv',
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k15/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k0.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k8.5/pose_data.csv',
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k15/pose_data.csv'
# ]

# # Generate simple labels for the CSV files
# #simple_labels = [f'csv_{i+1}' for i in range(len(csv_files))]
# simple_labels = ['v=1,k=0.5','v=1,k=8.5','v=1,k=15','v=1.5,k=0.5','v=1.5,k=8.5','v=1.5,k=15','v=2,k=0.5','v=2,k=8.5','v=2,k=15']

# # Function to calculate cross-correlation and determine delay
# def calculate_delay(series1, series2, time_step):
#     # Normalize the signals
#     series1 = (series1 - np.mean(series1)) / np.std(series1)
#     series2 = (series2 - np.mean(series2)) / np.std(series2)
    
#     # Cross-correlation
#     correlation = correlate(series1, series2, mode='full')
    
#     # Lags array
#     lags = np.arange(-len(series1) + 1, len(series2))
    
#     # Find the lag with the maximum cross-correlation
#     max_corr_index = np.argmax(correlation)
#     delay = lags[max_corr_index] * time_step  # Convert lag to time delay
    
#     return delay, correlation, lags

# # Initialize list to store delays
# delays = []

# # Time step (assuming constant sampling rate from your CSV file)
# time_step = 0.001  # Example: 1 ms time step; adjust based on your data

# # Loop through each CSV file to calculate delay
# for file in csv_files:
#     df = pd.read_csv(file)
    
#     # Calculate delay for each position/orientation component
#     delay_x, corr_x, lags_x = calculate_delay(df['EE Pose X'], df['Stylus Pose X'], time_step)
#     delay_y, corr_y, lags_y = calculate_delay(df['EE Pose Y'], df['Stylus Pose Y'], time_step)
#     delay_z, corr_z, lags_z = calculate_delay(df['EE Pose Z'], df['Stylus Pose Z'], time_step)
#     delay_roll, corr_roll, lags_roll = calculate_delay(df['EE Pose Roll'], df['Stylus Pose Roll'], time_step)
#     delay_pitch, corr_pitch, lags_pitch = calculate_delay(df['EE Pose Pitch'], df['Stylus Pose Pitch'], time_step)
#     delay_yaw, corr_yaw, lags_yaw = calculate_delay(df['EE Pose Yaw'], df['Stylus Pose Yaw'], time_step)
    
#     # Store the delays in a list
#     delays.append({
#         'File': file,
#         'Delay_X': delay_x,
#         'Delay_Y': delay_y,
#         'Delay_Z': delay_z,
#         'Delay_Roll': delay_roll,
#         'Delay_Pitch': delay_pitch,
#         'Delay_Yaw': delay_yaw
#     })

# # Convert the delays to a DataFrame
# delay_df = pd.DataFrame(delays)

# # Ensure delay_df DataFrame has columns: 'Delay_X', 'Delay_Y', 'Delay_Z', 'Delay_Roll', 'Delay_Pitch', 'Delay_Yaw'
# plt.figure(figsize=(15, 8))

# # Convert pandas Series to numpy array using .to_numpy() for all delays
# plt.plot(simple_labels, delay_df['Delay_X'].to_numpy(), marker='o', label='Delay X')
# plt.plot(simple_labels, delay_df['Delay_Y'].to_numpy(), marker='o', label='Delay Y')
# plt.plot(simple_labels, delay_df['Delay_Z'].to_numpy(), marker='o', label='Delay Z')

# # Add delays for Roll, Pitch, and Yaw
# plt.plot(simple_labels, delay_df['Delay_Roll'].to_numpy(), marker='o', label='Delay Roll')
# plt.plot(simple_labels, delay_df['Delay_Pitch'].to_numpy(), marker='o', label='Delay Pitch')
# plt.plot(simple_labels, delay_df['Delay_Yaw'].to_numpy(), marker='o', label='Delay Yaw')

# plt.xticks(rotation=45)
# plt.title('Delay Comparison (ElecLab_IITJWlan_Circle)')
# plt.xlabel('CSV Files')
# plt.ylabel('Delay (seconds)')
# plt.grid(True)
# plt.legend()
# plt.tight_layout()

# plt.show()

'''for RMSE (in six sublopts)'''

# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# # List of CSV filenames
# # csv_files = [
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k0.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k8.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k15/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k0.5/pose_data.csv',
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k8.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k15/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k0.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k8.5/pose_data.csv',
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k15/pose_data.csv'
# # ]

# csv_files = [
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k0.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k15/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k0.5/pose_data.csv',
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k15/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k0.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k8.5/pose_data.csv',
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k15/pose_data.csv'
# ]

# # Extract k values from file paths
# k_values = [0.5, 8.5, 15, 0.5, 8.5, 15, 0.5, 8.5, 15]

# # Extract velocity labels
# velocity_labels = [1, 1, 1, 1.5, 1.5, 1.5, 2, 2, 2]

# # Function to calculate RMSE
# def calculate_rmse(column1, column2):
#     return np.sqrt(((column1 - column2) ** 2).mean())

# # Initialize lists to store RMSE results
# rmse_x = []
# rmse_y = []
# rmse_z = []
# rmse_roll = []
# rmse_pitch = []
# rmse_yaw = []

# # Loop through each CSV file and calculate RMSE
# for file in csv_files:
#     df = pd.read_csv(file)
    
#     # Calculate RMSE for X, Y, Z positions
#     rmse_x.append(calculate_rmse(df['EE Pose X'], df['Stylus Pose X']))
#     rmse_y.append(calculate_rmse(df['EE Pose Y'], df['Stylus Pose Y']))
#     rmse_z.append(calculate_rmse(df['EE Pose Z'], df['Stylus Pose Z']))
    
#     # Calculate RMSE for Roll, Pitch, Yaw orientations
#     rmse_roll.append(calculate_rmse(df['EE Pose Roll'], df['Stylus Pose Roll']))
#     rmse_pitch.append(calculate_rmse(df['EE Pose Pitch'], df['Stylus Pose Pitch']))
#     rmse_yaw.append(calculate_rmse(df['EE Pose Yaw'], df['Stylus Pose Yaw']))

# # Convert RMSE lists to numpy arrays
# rmse_x = np.array(rmse_x)
# rmse_y = np.array(rmse_y)
# rmse_z = np.array(rmse_z)
# rmse_roll = np.array(rmse_roll)
# rmse_pitch = np.array(rmse_pitch)
# rmse_yaw = np.array(rmse_yaw)

# # Plot RMSE for X, Y, Z, Roll, Pitch, Yaw in six separate plots
# fig, axs = plt.subplots(2, 3, figsize=(18, 11))

# # Define plotting function for each subplot
# def plot_rmse(ax, rmse_values, ylabel):
#     ax.plot(k_values[:3], rmse_values[:3], marker='o', label='v=1')
#     ax.plot(k_values[3:6], rmse_values[3:6], marker='o', label='v=1.5')
#     ax.plot(k_values[6:], rmse_values[6:], marker='o', label='v=2')
#     ax.set_xlabel('k values')
#     ax.set_ylabel(ylabel)
#     ax.grid(True)
#     ax.legend()

# # Plot each RMSE on its respective subplot
# plot_rmse(axs[0, 0], rmse_x, 'RMSE X')
# plot_rmse(axs[0, 1], rmse_y, 'RMSE Y')
# plot_rmse(axs[0, 2], rmse_z, 'RMSE Z')
# plot_rmse(axs[1, 0], rmse_roll, 'RMSE Roll')
# plot_rmse(axs[1, 1], rmse_pitch, 'RMSE Pitch')
# plot_rmse(axs[1, 2], rmse_yaw, 'RMSE Yaw')

# # Customize the overall layout and show the plots
# fig.suptitle('RMSE comparison by k values for different velocities')
# plt.tight_layout(rect=[0, 0, 1, 0.95])
# plt.show()

'''for delay comparison (six diffrent plots)'''

# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.signal import correlate

# # List of CSV filenames and their labels
# # csv_files = [
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k0.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k8.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k15/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k0.5/pose_data.csv',
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k8.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k15/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k0.5/pose_data.csv', 
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k8.5/pose_data.csv',
# #     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k15/pose_data.csv'
# # ]

# csv_files = [
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k0.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1_k15/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k0.5/pose_data.csv',
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v1.5_k15/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k0.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k8.5/pose_data.csv',
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_triangle_v2_k15/pose_data.csv'
# ]

# # Labels for each CSV file
# simple_labels = ['v=1,k=0.5', 'v=1,k=8.5', 'v=1,k=15', 
#                  'v=1.5,k=0.5', 'v=1.5,k=8.5', 'v=1.5,k=15', 
#                  'v=2,k=0.5', 'v=2,k=8.5', 'v=2,k=15']

# # Extract the 'k' values
# k_values = [0.5, 8.5, 15] * 3

# # Function to calculate cross-correlation and determine delay
# def calculate_delay(series1, series2, time_diff):
#     # Normalize the signals
#     series1 = (series1 - np.mean(series1)) / np.std(series1)
#     series2 = (series2 - np.mean(series2)) / np.std(series2)
    
#     # Cross-correlation
#     correlation = correlate(series1, series2, mode='full')
    
#     # Lags array (account for time step differences)
#     lags = np.arange(-len(series1) + 1, len(series2))
    
#     # Find the lag with the maximum cross-correlation
#     max_corr_index = np.argmax(correlation)
#     delay = lags[max_corr_index] * time_diff  # Convert lag to time delay
    
#     return delay, correlation, lags

# # Initialize list to store delays
# delays = {'X': [], 'Y': [], 'Z': [], 'Roll': [], 'Pitch': [], 'Yaw': []}

# # Loop through each CSV file to calculate delay
# for file in csv_files:
#     df = pd.read_csv(file)
    
#     # Compute time step differences (delta time for each row)
#     time_diff = np.mean(np.diff(df['Time']))

#     # Calculate delay for each position/orientation component
#     delay_x, _, _ = calculate_delay(df['EE Pose X'], df['Stylus Pose X'], time_diff)
#     delay_y, _, _ = calculate_delay(df['EE Pose Y'], df['Stylus Pose Y'], time_diff)
#     delay_z, _, _ = calculate_delay(df['EE Pose Z'], df['Stylus Pose Z'], time_diff)
#     delay_roll, _, _ = calculate_delay(df['EE Pose Roll'], df['Stylus Pose Roll'], time_diff)
#     delay_pitch, _, _ = calculate_delay(df['EE Pose Pitch'], df['Stylus Pose Pitch'], time_diff)
#     delay_yaw, _, _ = calculate_delay(df['EE Pose Yaw'], df['Stylus Pose Yaw'], time_diff)
    
#     # Store the delays in the respective lists
#     delays['X'].append(delay_x)
#     delays['Y'].append(delay_y)
#     delays['Z'].append(delay_z)
#     delays['Roll'].append(delay_roll)
#     delays['Pitch'].append(delay_pitch)
#     delays['Yaw'].append(delay_yaw)

# # Plotting the results
# fig, axes = plt.subplots(2, 3, figsize=(15, 10))

# # Titles for each subplot
# titles = ['Delay in X', 'Delay in Y', 'Delay in Z', 'Delay in Roll', 'Delay in Pitch', 'Delay in Yaw']

# # Map delays to respective axes
# delay_components = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']

# # Loop over each subplot
# for i, ax in enumerate(axes.flatten()):
#     component = delay_components[i]
#     # Plot the data, grouping by v values
#     ax.plot(k_values[:3], delays[component][:3], marker='o', label='v=1')
#     ax.plot(k_values[3:6], delays[component][3:6], marker='o', label='v=1.5')
#     ax.plot(k_values[6:], delays[component][6:], marker='o', label='v=2')

#     # Set titles, labels, and legends
#     ax.set_title(titles[i])
#     ax.set_xlabel('k value')
#     ax.set_ylabel('Delay (seconds)')
#     ax.legend()
#     ax.grid(True)

# # Adjust layout
# plt.tight_layout()
# plt.show()

'''update for rmse for 22 september experiments'''

# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# # List of CSV filenames
# csv_files = [ 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/22_sept_2024/RoboLab_hussernet_circle_v1_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/22_sept_2024/RoboLab_hussernet_circle_v1.5_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v2_k8.5/pose_data.csv', 
#     '/home/user/experiment_data/22_sept_2024/RoboLab_hussernet_circle_v2_k8.5/pose_data.csv'
#     ]

# # Extract k values from file paths
# k_values = [8.5, 8.5, 8.5, 8.5, 8.5, 8.5]

# # Extract velocity labels
# velocity_labels = [1, 1, 1.5, 1.5, 2, 2]

# # Function to calculate RMSE
# def calculate_rmse(column1, column2):
#     return np.sqrt(((column1 - column2) ** 2).mean())

# # Initialize lists to store RMSE results
# rmse_x = []
# rmse_y = []
# rmse_z = []
# rmse_roll = []
# rmse_pitch = []
# rmse_yaw = []

# # Loop through each CSV file and calculate RMSE
# for file in csv_files:
#     df = pd.read_csv(file)
    
#     # Calculate RMSE for X, Y, Z positions
#     rmse_x.append(calculate_rmse(df['EE Pose X'], df['Stylus Pose X']))
#     rmse_y.append(calculate_rmse(df['EE Pose Y'], df['Stylus Pose Y']))
#     rmse_z.append(calculate_rmse(df['EE Pose Z'], df['Stylus Pose Z']))
    
#     # Calculate RMSE for Roll, Pitch, Yaw orientations
#     rmse_roll.append(calculate_rmse(df['EE Pose Roll'], df['Stylus Pose Roll']))
#     rmse_pitch.append(calculate_rmse(df['EE Pose Pitch'], df['Stylus Pose Pitch']))
#     rmse_yaw.append(calculate_rmse(df['EE Pose Yaw'], df['Stylus Pose Yaw']))

# # Convert RMSE lists to numpy arrays
# rmse_x = np.array(rmse_x)
# rmse_y = np.array(rmse_y)
# rmse_z = np.array(rmse_z)
# rmse_roll = np.array(rmse_roll)
# rmse_pitch = np.array(rmse_pitch)
# rmse_yaw = np.array(rmse_yaw)

# # Plot RMSE for X, Y, Z, Roll, Pitch, Yaw in six separate plots
# fig, axs = plt.subplots(2, 3, figsize=(18, 11))

# # Define plotting function for each subplot
# # def plot_rmse(ax, rmse_values, ylabel):
# #     ax.plot(np.array([velocity_labels[0], velocity_labels[2], velocity_labels[4]]), np.array([rmse_values[0], rmse_values[2], rmse_values[4]]), marker='o', label='ElecLab(iitjwlan)')
# #     ax.plot(np.array([velocity_labels[1], velocity_labels[3], velocity_labels[5]]), np.array([rmse_values[1], rmse_values[3], rmse_values[5]]), marker='o', label='RoboLab(hussernet)')
# #     ax.set_xlabel('velocity')
# #     ax.set_ylabel(ylabel)
# #     ax.grid(True)
# #     ax.legend()

# # just for report
# def plot_rmse(ax, rmse_values, ylabel):
#     ax.plot(np.array([velocity_labels[0], velocity_labels[2], velocity_labels[4]]), np.array([rmse_values[0], rmse_values[2], rmse_values[4]]), marker='o', label='RoboLab(hussernet)')
#     ax.plot(np.array([velocity_labels[1], velocity_labels[3], velocity_labels[5]]), np.array([rmse_values[1], rmse_values[3], rmse_values[5]]), marker='o', label='ElecLab(iitjwlan)')
#     ax.set_xlabel('velocity')
#     ax.set_ylabel(ylabel)
#     ax.grid(True)
#     ax.legend()

# # Plot each RMSE on its respective subplot
# plot_rmse(axs[0, 0], rmse_x, 'RMSE X')
# plot_rmse(axs[0, 1], rmse_y, 'RMSE Y')
# plot_rmse(axs[0, 2], rmse_z, 'RMSE Z')
# plot_rmse(axs[1, 0], rmse_roll, 'RMSE Roll')
# plot_rmse(axs[1, 1], rmse_pitch, 'RMSE Pitch')
# plot_rmse(axs[1, 2], rmse_yaw, 'RMSE Yaw')

# # Customize the overall layout and show the plots
# fig.suptitle('RMSE comparison by k values for different velocities')
# plt.tight_layout(rect=[0, 0, 1, 0.95])
# plt.show()

'''3D view for the paper'''

# import pandas as pd
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # Load the data
# file_path = '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k8.5/pose_data.csv'
# df = pd.read_csv(file_path)

# # Extract the columns for end effector and stylus positions
# ee_x = df['EE Pose X'].to_numpy()
# ee_y = df['EE Pose Y'].to_numpy()
# ee_z = df['EE Pose Z'].to_numpy()

# stylus_x = df['Stylus Pose X'].to_numpy()
# stylus_y = df['Stylus Pose Y'].to_numpy()
# stylus_z = df['Stylus Pose Z'].to_numpy()

# # Create a 3D plot
# fig = plt.figure(figsize=(8, 6))  # Adjust figure size
# ax = fig.add_subplot(111, projection='3d')

# # Plot the trajectories of the end-effector and stylus
# ax.plot(ee_x, ee_y, ee_z, label='End-Effector Trajectory', color='b')
# ax.plot(stylus_x, stylus_y, stylus_z, label='Stylus Trajectory', color='r')

# # Set labels
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')

# # Set title
# ax.set_title('End-Effector and Stylus Trajectory')

# # Customize ticks (optional)
# ax.tick_params(axis='both', which='both', direction='in')

# # Add legend
# ax.legend()

# # Remove grid
# plt.grid(False)

# # Set the background color to white
# fig.patch.set_facecolor('white')
# ax.set_facecolor('white')

# # Display the plot
# plt.show()

'''RMSE plot for the paper'''

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the data
file_path = '/home/user/experiment_data/06_sept_2024/ElecLab_iitjwlan_circle_v1.5_k8.5/pose_data.csv'
df = pd.read_csv(file_path)

# Extract the columns for end effector and stylus positions
ee_x = df['EE Pose X'].to_numpy()
ee_y = df['EE Pose Y'].to_numpy()
ee_z = df['EE Pose Z'].to_numpy()

stylus_x = df['Stylus Pose X'].to_numpy()
stylus_y = df['Stylus Pose Y'].to_numpy()
stylus_z = df['Stylus Pose Z'].to_numpy()

# Calculate RMSE for each axis (X, Y, Z)
rmse_x = np.sqrt(np.mean((ee_x - stylus_x) ** 2))
rmse_y = np.sqrt(np.mean((ee_y - stylus_y) ** 2))
rmse_z = np.sqrt(np.mean((ee_z - stylus_z) ** 2))

# Combine the RMSE values
rmse_values = [rmse_x, rmse_y, rmse_z]
axes = ['X', 'Y', 'Z']

# Plot RMSE values
plt.figure(figsize=(6, 4))
plt.bar(axes, rmse_values, color=['b', 'g', 'r'])

# Set title and labels
plt.title('RMSE for End-Effector and Stylus Trajectory')
plt.xlabel('Axis')
plt.ylabel('RMSE')

# Show the plot
plt.show()





