import numpy as np
import matplotlib.pyplot as plt
import os

# Paths
output_folder = "A1/output/task2"  
os.makedirs(output_folder, exist_ok=True)  # Make sure output folder exists

# Constants
N = 250
L = 50
main_time_steps = 5000
last_cycle_steps = 50
runs_per_r = 50
r_values = np.arange(0.025, 1.425, 0.025)  # From 0.025 to 1.4

# To store average amplitude for each r
average_amplitudes = []

for r in r_values:
    amplitudes = []

    for run in range(runs_per_r):
        # 1. Generate firefly positions and clocks
        positions = np.random.rand(N, 2)
        clocks = np.random.randint(0, L, size=N)

        # 2. Precompute neighbors
        neighbors = []
        for i in range(N):
            dist = np.linalg.norm(positions - positions[i], axis=1)
            neighbor_ids = np.where((dist < r) & (dist > 0))[0]  # exclude self
            neighbors.append(neighbor_ids)

        # 3. Run main simulation for 5000 steps
        for t in range(main_time_steps):
            flashing = (clocks < (L // 2))

            # Synchronization correction
            for i in range(N):
                if clocks[i] == 0:
                    neighbor_ids = neighbors[i]
                    if len(neighbor_ids) > 0:
                        flashing_neighbors = flashing[neighbor_ids]
                        if np.sum(flashing_neighbors) > len(neighbor_ids) / 2:
                            clocks[i] = (clocks[i] + 1) % L

            clocks = (clocks + 1) % L

        # 4. After 5000 steps, record flashing for last 50 steps
        flashing_counts_last_cycle = []

        for t in range(last_cycle_steps):
            flashing = (clocks < (L // 2))
            flashing_counts_last_cycle.append(np.sum(flashing))

            # Synchronization correction in last steps (optional depending on interpretation)
            for i in range(N):
                if clocks[i] == 0:
                    neighbor_ids = neighbors[i]
                    if len(neighbor_ids) > 0:
                        flashing_neighbors = flashing[neighbor_ids]
                        if np.sum(flashing_neighbors) > len(neighbor_ids) / 2:
                            clocks[i] = (clocks[i] + 1) % L

            clocks = (clocks + 1) % L

        # 5. Calculate amplitude
        min_flash = np.min(flashing_counts_last_cycle)
        max_flash = np.max(flashing_counts_last_cycle)
        amplitude = (max_flash - min_flash) / 2

        amplitudes.append(amplitude)

    # 6. Average amplitude over 50 runs
    average_amplitudes.append(np.mean(amplitudes))
    print(f"Finished r={r:.3f}")

# 7. Plot amplitude vs r
title = "Average Flashing Amplitude vs Vicinity Distance"
plt.figure(figsize=(12, 6))
plt.plot(r_values, average_amplitudes, marker='o')
plt.title(title)
plt.xlabel("Vicinity Distance r")
plt.ylabel("Average Amplitude of Flash Cycle")
plt.grid()
# Save the plot
filename = title.replace(' ', '_').replace('=', '').replace('(', '').replace(')', '') + '.png'
plt.savefig(os.path.join(output_folder, filename))
plt.show()
