#!/usr/bin/env python3

import pandas as pd

# Read the CSV file with headers
df = pd.read_csv('/home/user/experiment_data/standard3/pose_data_for_exp_interpolated.csv')

# Divide all the values in the first column by 1.5, skipping the header
df.iloc[:, 0] = df.iloc[:, 0] / 2

# Save the modified DataFrame to a new CSV file
df.to_csv('/home/user/experiment_data/standard3_v2/pose_data_for_exp_interpolated.csv', index=False)

print("Values in the first column have been divided by 1.5 and saved to 'output.csv'.")
