import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('/home/autonomous-lab/Desktop/delay/final_master/0/posed.csv')

timestamps = df.iloc[:, 0].values
delays = df.iloc[:, 1].values * 1000  # Convert delays to ms

avg_delay_ms = delays.mean()

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




# import pandas as pd
# import numpy as np
# import os
# import ast
# import matplotlib.pyplot as plt

# folder_path = "/home/autonomous-lab/Desktop/delay/2AFC/14"

# all_pairs = []
# all_yes = []

# for file in os.listdir(folder_path):
#     if file.endswith(".csv"):
#         file_path = os.path.join(folder_path, file)
#         data = pd.read_csv(file_path)
        
#         for i in range(len(data)):
#             pair = ast.literal_eval(data.loc[i, "AsymPair"])
#             pair_tuple = tuple(pair)   # or tuple(sorted(pair))
            
#             is_yes = 1 if data.loc[i, "UserAnswer"] == 'Y' else 0
            
#             all_pairs.append(pair_tuple)
#             all_yes.append(is_yes)

# df = pd.DataFrame({
#     "pair": all_pairs,
#     "yes": all_yes
# })

# result = df.groupby("pair")["yes"].mean().reset_index()
# result.columns = ["DelayPair", "P_Yes"]

# print(result)

# # -------- FIX + LINE PLOT --------
# x = np.arange(len(result))
# y = result["P_Yes"].to_numpy()   

# plt.figure()
# plt.plot(x, y, marker='o')
# plt.xticks(x, result["DelayPair"], rotation=45)
# plt.xlabel("Delay Combination")
# plt.ylabel("P(Yes)")
# plt.title("Probability of Yes for Each Delay Combination")
# plt.grid(True)
# plt.tight_layout()
# plt.show()