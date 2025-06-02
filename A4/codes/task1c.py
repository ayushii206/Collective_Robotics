import numpy as np
import matplotlib.pyplot as plt
import os

# Parameters
N = 20
T = 500
r = 0.045
P = 0.015
v = [0.001, -0.001]
runs = 100  # You can increase to 1000 locally for better results

# Initialize matrices
A = np.zeros((N + 1, N + 1), dtype=int)
M = np.zeros(N + 1, dtype=int)

def simulate_run():
    pos = np.random.rand(N)
    dir = np.random.choice([-1, 1], size=N)
    lefts = []

    for _ in range(T):
        lefts.append(np.sum(dir == -1))
        new_dir = dir.copy()

        for i in range(N):
            dists = np.abs(pos - pos[i])
            dists = np.minimum(dists, 1 - dists)
            neighbors = (dists < r) & (dists > 0)
            opposite = np.sum(dir[neighbors] == -dir[i])
            same = np.sum(dir[neighbors] == dir[i])

            if opposite > same:
                new_dir[i] = -dir[i]
            elif np.random.rand() < P:
                new_dir[i] = -dir[i]

        dir = new_dir
        pos = (pos + 0.001 * dir) % 1

    return lefts

# Build histogram and counts
for _ in range(runs):
    L = simulate_run()
    for t in range(T - 1):
        A[L[t]][L[t + 1]] += 1
        M[L[t]] += 1

# Normalize transition matrix
P_matrix = np.zeros_like(A, dtype=float)
for i in range(N + 1):
    if M[i] > 0:
        P_matrix[i] = A[i] / M[i]

# Sample trajectory from transition probabilities
def sample_trajectory():
    traj = []
    L = np.random.randint(0, N + 1)
    traj.append(L)
    for _ in range(T - 1):
        probs = P_matrix[L]
        if probs.sum() == 0:
            next_L = L
        else:
            probs = probs / probs.sum()
            next_L = np.random.choice(np.arange(N + 1), p=probs)
        traj.append(next_L)
        L = next_L
    return traj

# Generate trajectories
simulated_L = simulate_run()
sampled_L = sample_trajectory()

# Paths
output_folder = "A4/output/task1"  
os.makedirs(output_folder, exist_ok=True)

# Plot
plt.figure(figsize=(12, 5))
plt.plot(simulated_L, label="Task A: Simulated L(t)", linewidth=1.8)
plt.plot(sampled_L, label="Task C: Sampled from P(L)", linestyle='--', linewidth=1.5)
plt.xlabel("Time Step")
plt.ylabel("Number of Left-going Locusts (L)")
plt.title("Comparison of Task A and Task C Trajectories")
plt.legend()
plt.grid(True)
plt.tight_layout()
# Save the figure
filename = "task1c.png"
filepath = os.path.join(output_folder, filename)
plt.savefig(filepath)
plt.show()
