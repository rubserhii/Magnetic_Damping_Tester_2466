# plot_velocity.py

import pandas as pd
import matplotlib.pyplot as plt

LOG_FILE = 'velocity_log.csv'

# --- Load Data ---
df = pd.read_csv(LOG_FILE, parse_dates=['timestamp'])

# --- Plot ---
plt.figure(figsize=(10, 4))
plt.plot(df['timestamp'], df['velocity'], linestyle='-', marker='.')
plt.title("Velocity Over Time")
plt.xlabel("Time")
plt.ylabel("Velocity")
plt.grid(True)
plt.tight_layout()
plt.show()