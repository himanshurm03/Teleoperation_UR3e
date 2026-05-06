# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np

# # Load CSV file
# file_path = '/home/autonomous-lab/Desktop/delay/force_limiting.csv'  # change if needed
# data = pd.read_csv(file_path)

# # Extract columns (0-based indexing)
# time = data.iloc[:, 0].astype(float).to_numpy()          # 1st column
# force_received = data.iloc[:, 3].astype(float).to_numpy() # 4th column
# force_applied = data.iloc[:, 6].astype(float).to_numpy()  # 7th column

# # Remove NaNs safely
# mask = ~np.isnan(time) & ~np.isnan(force_received) & ~np.isnan(force_applied)
# time = time[mask]
# force_received = force_received[mask]
# force_applied = force_applied[mask]

# # Plot
# plt.figure()
# plt.plot(time, force_received, label='Force Received')
# plt.plot(time, force_applied, label='Force Applied to Haptic Device')

# plt.xlabel('Time (s)')
# plt.ylabel('Force')
# plt.title('Force Comparison vs Time')
# plt.legend()
# plt.grid(True)
# plt.show()


import csv

def count_correct_responses(csv_file_path):
    correct_count = 0
    total_count = 0

    with open(csv_file_path, 'r') as file:
        reader = csv.reader(file)

        # If your CSV has a header, uncomment the next line
        # next(reader)

        for row in reader:
            try:
                correct_data = row[9]   # Column 10 (index starts from 0)
                user_data = row[10]     # Column 11

                if correct_data == user_data:
                    correct_count += 1

                total_count += 1

            except IndexError:
                print("Skipping row due to missing columns:", row)

    return correct_count, total_count


# ---- Usage ----
file_path = "/home/autonomous-lab/Desktop/delay/himanshuuu_2026-03-27_17-43-47.csv"  # Replace with your CSV file path

correct, total = count_correct_responses(file_path)

print(f"Total Experiments: {total}")
print(f"Correct Responses: {correct}")
print(f"Accuracy: {(correct/total)*100:.2f}%")
