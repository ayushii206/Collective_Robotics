import numpy as np
import matplotlib.pyplot as plt
import os

# Constants
N = 250
L = 50
time_steps = 5000
r_values = [0.05, 0.1, 0.5, 1.4]

# Paths
output_folder = "output/task2"  
os.makedirs(output_folder, exist_ok=True)  # Make sure output folder exists

# Generating firefly positions
positions = np.random.rand(N, 2)  # Random (x, y) in [0, 1]

# Initializing internal clocks
clocks = np.random.randint(0, L, size=N)

for r in r_values:
    # 3. Compute neighbors
    neighbors = []
    for i in range(N):
        dist = np.linalg.norm(positions - positions[i], axis=1)
        neighbor_ids = np.where((dist < r) & (dist > 0))[0]  # exclude self (dist > 0)
        neighbors.append(neighbor_ids)

    # 4. Calculate average number of neighbors
    avg_neighbors = np.mean([len(n) for n in neighbors])
    print(f"r={r}: Average neighbors = {avg_neighbors}")

    # 5. Simulate over time
    clocks_copy = clocks.copy()  # Work on a copy so initial clocks stay same for different r
    flashing_count = []

    for t in range(time_steps):
        flashing = (clocks_copy < (L // 2))  # Flashing if clock in [0, 24]
        flashing_count.append(np.sum(flashing))

        # Synchronization correction
        for i in range(N):
            if clocks_copy[i] == 0:  # just started flashing
                neighbor_ids = neighbors[i]
                if len(neighbor_ids) > 0:
                    flashing_neighbors = flashing[neighbor_ids]
                    if np.sum(flashing_neighbors) > len(neighbor_ids) / 2:
                        clocks_copy[i] = (clocks_copy[i] + 1) % L  # correct cycle
        # Increment clocks
        clocks_copy = (clocks_copy + 1) % L

    # 6. Plotting
    plt.figure()
    plt.plot(flashing_count)
    plt.title(f"Flashing Fireflies Over Time (r={r})")
    plt.xlabel("Time Step")
    plt.ylabel("Number of Flashing Fireflies")
    plt.ylim(0, 150)
    plt.grid()
    # Save the plot
    filename = f"fireflies_r_{str(r).replace('.', '_')}.png"
    filepath = os.path.join(output_folder, filename)
    plt.savefig(filepath)
    plt.show()
