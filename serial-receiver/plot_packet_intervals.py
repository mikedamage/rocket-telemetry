#!/usr/bin/env python3
"""
Plot telemetry packet intervals over time.
Calculates the delta between consecutive timestamps to show packet timing consistency.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the CSV data
df = pd.read_csv('/Users/mike/dev/rocket-telemetry/serial-receiver/dump.csv')

# Calculate packet intervals (delta between consecutive timestamps)
intervals = df['timestamp'].diff().dropna().values  # in milliseconds

# Create time axis normalized to start at 0 (in seconds)
# Use cumulative time from the timestamps, excluding first row
timestamps = df['timestamp'].values[1:]  # Skip first since we have no interval for it
time_seconds = (timestamps - timestamps[0]) / 1000.0  # Convert to seconds from start

# Apply publication style
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial', 'Helvetica', 'DejaVu Sans']
plt.rcParams['font.size'] = 10
plt.rcParams['axes.labelsize'] = 11
plt.rcParams['axes.titlesize'] = 12

# Create figure
fig, ax = plt.subplots(figsize=(10, 5))

# Plot packet intervals
ax.plot(time_seconds, intervals, linewidth=0.5, alpha=0.7, color='#0072B2')

# Add reference line at expected 20ms interval
ax.axhline(y=20, color='#009E73', linestyle='--', linewidth=1.5,
           label='Expected (20 ms)', alpha=0.8)

# Calculate and display statistics
mean_interval = np.mean(intervals)
std_interval = np.std(intervals)
median_interval = np.median(intervals)
max_interval = np.max(intervals)
min_interval = np.min(intervals)

# Add mean line
ax.axhline(y=mean_interval, color='#D55E00', linestyle=':', linewidth=1.5,
           label=f'Mean ({mean_interval:.1f} ms)', alpha=0.8)

# Labels and title
ax.set_xlabel('Time Since Start (seconds)')
ax.set_ylabel('Packet Interval (ms)')
ax.set_title('Telemetry Packet Interval Over Time')

# Set y-axis to show relevant range with some padding
y_max = min(max_interval * 1.1, 200)  # Cap at 200ms for readability
ax.set_ylim(0, y_max)

# Add legend with statistics
stats_text = f'n={len(intervals):,}\nMean: {mean_interval:.1f} ms\nMedian: {median_interval:.1f} ms\nStd: {std_interval:.1f} ms\nMin: {min_interval:.0f} ms\nMax: {max_interval:.0f} ms'
ax.text(0.98, 0.97, stats_text, transform=ax.transAxes, fontsize=9,
        verticalalignment='top', horizontalalignment='right',
        bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

ax.legend(loc='upper left', framealpha=0.9)

# Remove top and right spines
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

plt.tight_layout()

# Save figure
output_path = '/Users/mike/dev/rocket-telemetry/serial-receiver/packet_intervals.png'
plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
print(f"Figure saved to: {output_path}")

# Also show summary
print(f"\n=== Packet Interval Statistics ===")
print(f"Total packets: {len(df):,}")
print(f"Duration: {time_seconds[-1]:.1f} seconds")
print(f"Mean interval: {mean_interval:.2f} ms")
print(f"Median interval: {median_interval:.2f} ms")
print(f"Std deviation: {std_interval:.2f} ms")
print(f"Min interval: {min_interval:.0f} ms")
print(f"Max interval: {max_interval:.0f} ms")

# Count outliers (packets with interval > 50ms)
outliers = intervals[intervals > 50]
print(f"\nPackets with interval > 50ms: {len(outliers)} ({100*len(outliers)/len(intervals):.2f}%)")

# plt.show()  # Commented out for non-interactive use
