import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

N = 20                      # Number of locusts
steps = 500                 # Time steps
speed = 0.001               # Movement speed
r = 0.045                   # Perception range
P = 0.015                   # Spontaneous switch probability

positions = np.random.rand(N)                              # Positions on [0, 1)
directions = np.random.choice([-1, 1], size=N)             # -1 = left, +1 = right
left_counts = []                                           # Track number of left-goers

history_positions = [positions.copy()]
history_directions = [directions.copy()]

for _ in range(steps):
    new_directions = directions.copy()
    
    for i in range(N):
        dists = np.abs(positions - positions[i])
        dists = np.minimum(dists, 1 - dists)               # Ring distance
        neighbors_idx = (dists < r) & (dists > 0)
        neighbor_dirs = directions[neighbors_idx]
        
        if len(neighbor_dirs) > 0:
            if np.sum(neighbor_dirs == -directions[i]) > len(neighbor_dirs) / 2:
                new_directions[i] = -directions[i]
            elif np.random.rand() < P:
                new_directions[i] = -directions[i]
        elif np.random.rand() < P:
            new_directions[i] = -directions[i]
    
    directions = new_directions
    positions = (positions + speed * directions) % 1       # Move on ring
    
    left_counts.append(np.sum(directions == -1))
    history_positions.append(positions.copy())
    history_directions.append(directions.copy())

plt.figure(figsize=(10, 4))
plt.plot(left_counts, color='blue', label="Left-going Locusts")
plt.title("Number of Left-going Locusts Over Time")
plt.xlabel("Time Step")
plt.ylabel("Count")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

fig, ax = plt.subplots(figsize=(6, 6))
circle = plt.Circle((0.5, 0.5), 0.45, color='lightgrey', fill=False)
scat = ax.scatter([], [], c=[], cmap='bwr', s=100)

def init():
    ax.add_patch(circle)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect('equal')
    ax.axis('off')
    return scat,

def update(frame):
    angles = history_positions[frame] * 2 * np.pi
    x = 0.5 + 0.45 * np.cos(angles)
    y = 0.5 + 0.45 * np.sin(angles)
    colors = history_directions[frame]
    scat.set_offsets(np.c_[x, y])
    scat.set_array(colors)
    return scat,

ani = FuncAnimation(fig, update, frames=len(history_positions), init_func=init, blit=True)
plt.show()
from IPython.display import HTML
HTML(ani.to_jshtml())
