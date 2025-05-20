import numpy as np
import matplotlib.pyplot as plt
import os

# Parameters
NUM_ROBOTS = 60
ANTI_AGENT_COUNTS = [0, 2, 4, 6, 8, 10, 12, 15]
NEIGHBOR_RADIUS = 5
LEAVE_THRESHOLD = 4
WORLD_SIZE = 100
MOVE_SPEED = 1.5
FRAMES = 800
REPEATS = 30
TIMEOUT_FRAMES = 100  # Memory-based unstop timeout

# Define robot class
class Robot:
    def __init__(self, is_anti=False):
        self.x = np.random.uniform(0, WORLD_SIZE)
        self.y = np.random.uniform(0, WORLD_SIZE)
        self.is_anti = is_anti
        self.stopped = False
        self.stop_time = 0

    def move(self):
        if not self.stopped or self.is_anti:
            angle = np.random.uniform(0, 2 * np.pi)
            self.x += np.cos(angle) * MOVE_SPEED
            self.y += np.sin(angle) * MOVE_SPEED
            self.x = np.clip(self.x, 0, WORLD_SIZE)
            self.y = np.clip(self.y, 0, WORLD_SIZE)

    def distance_to(self, other):
        return np.hypot(self.x - other.x, self.y - other.y)

# Function to compute largest cluster size using DFS
def get_largest_cluster_size(robots):
    visited = set()
    max_cluster = 0

    def dfs(r, cluster):
        for other in robots:
            if other not in visited and other.stopped and r.distance_to(other) < NEIGHBOR_RADIUS:
                visited.add(other)
                cluster.append(other)
                dfs(other, cluster)

    for r in robots:
        if r.stopped and r not in visited:
            cluster = [r]
            visited.add(r)
            dfs(r, cluster)
            max_cluster = max(max_cluster, len(cluster))

    return max_cluster

# Run a single simulation
def run_simulation(num_anti_agents):
    robots = [Robot() for _ in range(NUM_ROBOTS)]
    anti_agents = [Robot(is_anti=True) for _ in range(num_anti_agents)]

    for _ in range(FRAMES):
        # Update robots
        for robot in robots:
            if not robot.stopped:
                neighbors = [
                    r for r in robots
                    if r != robot and robot.distance_to(r) < NEIGHBOR_RADIUS
                ]
                p_stop = min(1.0, len(neighbors) / 5)
                if np.random.rand() < p_stop:
                    robot.stopped = True
                    robot.stop_time = 0
            else:
                robot.stop_time += 1
                if robot.stop_time >= TIMEOUT_FRAMES:
                    robot.stopped = False

            robot.move()

        # Anti-agent broadcasting
        for anti in anti_agents:
            anti.move()
            for robot in robots:
                if robot.stopped and anti.distance_to(robot) < NEIGHBOR_RADIUS:
                    neighbors = [
                        r for r in robots
                        if r != robot and r.stopped and anti.distance_to(r) < NEIGHBOR_RADIUS
                    ]
                    if len(neighbors) + 1 >= LEAVE_THRESHOLD:
                        robot.stopped = False

    return get_largest_cluster_size(robots)

# Run experiments for different anti-agent counts
results = {k: [] for k in ANTI_AGENT_COUNTS}
for k in ANTI_AGENT_COUNTS:
    for _ in range(REPEATS):
        cluster_size = run_simulation(k)
        results[k].append(cluster_size)

# Process results
avg_results = [np.mean(results[k]) for k in ANTI_AGENT_COUNTS]
std_results = [np.std(results[k]) for k in ANTI_AGENT_COUNTS]
percentages = [100 * k / NUM_ROBOTS for k in ANTI_AGENT_COUNTS]

# Paths
output_folder = "A3/output/task2"  
os.makedirs(output_folder, exist_ok=True)

# Plot
plt.figure(figsize=(8, 5))
plt.errorbar(percentages, avg_results, yerr=std_results, fmt='-o', capsize=5)
plt.title("Improved Robot Aggregation with Anti-Agent Percentage")
plt.xlabel("Anti-Agent Percentage (%)")
plt.ylabel("Average Largest Cluster Size")
plt.grid(True)
plt.tight_layout()
# Save the plot
filename = "task2c.png"
filepath = os.path.join(output_folder, filename)
plt.savefig(filepath)
plt.show()
