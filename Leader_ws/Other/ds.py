import os
import pandas as pd
import matplotlib.pyplot as plt

folder_path = r'/home/autonomous-lab/Desktop/delay/Packetloss/1'

files = [f for f in os.listdir(folder_path) if f.endswith('.csv')]

all_pairs = []
all_ratings = []

for file in files:
    file_path = os.path.join(folder_path, file)
    T = pd.read_csv(file_path)

    for j in range(len(T)):
        # Example string: "(1,15)"
        s = T.loc[j, 'AsymPair']

        # Remove parentheses
        s = s.replace('(', '').replace(')', '')

        # Split by comma
        parts = s.split(',')

        a = float(parts[0])
        b = float(parts[1])

        # Store key
        key = f"{int(a)}_{int(b)}"

        all_pairs.append(key)
        all_ratings.append(T.loc[j, 'Rating'])

# -----------------------------
# Unique combinations
# -----------------------------
df = pd.DataFrame({
    'pair': all_pairs,
    'rating': all_ratings
})

avg_df = df.groupby('pair')['rating'].mean().reset_index()

unique_pairs = avg_df['pair'].tolist()
avg_rating = avg_df['rating'].tolist()

# Labels
labels = []
for pair in unique_pairs:
    parts = pair.split('_')
    labels.append(f"({parts[0]},{parts[1]})")

# -----------------------------
# Plot
# -----------------------------
plt.figure()
plt.plot(avg_rating, '-o', linewidth=2)

plt.xticks(range(len(labels)), labels, rotation=45)

plt.xlabel('Delay Combination')
plt.ylabel('Average Rating')
plt.title('Average Dissimilarity Rating')

plt.grid(True)
plt.tight_layout()

plt.show()