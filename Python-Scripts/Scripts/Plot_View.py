import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
filename = "drone_flight_log_20251101_181209.csv"
df = pd.read_csv(filename)

# Ensure required columns exist
required_cols = ['Timestamp (s)', 'Integrated Position X (m)', 'Integrated Position Y (m)', 'Height (m)']
for col in required_cols:
    if col not in df.columns:
        raise ValueError(f"Missing column: {col}")

# === Plot 1: Integrated Position (Dead Reckoning) ===
plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.plot(-df['Integrated Position X (m)'], df['Integrated Position Y (m)'], 'b-', linewidth=2, label='Trajectory')
plt.plot(0, 0, 'ko', markersize=8, markerfacecolor='yellow', markeredgecolor='black', label='Start (Origin)')
plt.plot(-df['Integrated Position X (m)'].iloc[-1], df['Integrated Position Y (m)'].iloc[-1], 'ro', markersize=8, label='End')
plt.title('Integrated Position (Dead Reckoning)')
plt.xlabel('X Position (m) [flipped for visualization]')
plt.ylabel('Y Position (m)')
plt.axis('equal')
plt.grid(True, alpha=0.3)
plt.legend()

# === Plot 2: Height over Time ===
plt.subplot(1, 2, 2)
plt.plot(df['Timestamp (s)'], df['Height (m)'], 'orange', linewidth=2)
plt.title('Height Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Height (m)')
plt.grid(True, alpha=0.3)

# Optional: Add target height line if known (e.g., 0.3 m)
target_height = 0.3
plt.axhline(y=target_height, color='red', linestyle='--', alpha=0.7, label=f'Target = {target_height}m')
plt.legend()

# Adjust layout and show
plt.tight_layout()
plt.show()