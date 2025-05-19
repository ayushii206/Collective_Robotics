import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parameters
NUM_ROBOTS = 30
NUM_ANTI_AGENTS = 3
NEIGHBOR_RADIUS = 5
LEAVE_THRESHOLD = 4  # Cluster size threshold (including the robot itself)
WORLD_SIZE = 100
STOP_PROB = 0.1
MOVE_SPEED = 1.5

# Define robot class
class Robot:
    def __init__(self, is_anti=False):
        self.x = np.random.uniform(0, WORLD_SIZE)
        self.y = np.random.uniform(0, WORLD_SIZE)
        self.is_anti = is_anti
        self.stopped = False

    def move(self):
        if not self.stopped or self.is_anti:
            angle = np.random.uniform(0, 2 * np.pi)
            self.x += np.cos(angle) * MOVE_SPEED
            self.y += np.sin(angle) * MOVE_SPEED
            self.x = np.clip(self.x, 0, WORLD_SIZE)
            self.y = np.clip(self.y, 0, WORLD_SIZE)

    def distance_to(self, other):
        return np.hypot(self.x - other.x, self.y - other.y)

# Initialize agents
robots = [Robot() for _ in range(NUM_ROBOTS)]
anti_agents = [Robot(is_anti=True) for _ in range(NUM_ANTI_AGENTS)]
all_agents = robots + anti_agents

# Simulation update function
def update(frame):
    plt.clf()
    x_coords, y_coords, colors = [], [], []

    # Update normal robot behavior
    for robot in robots:
        if not robot.stopped:
            neighbors = [
                r for r in robots
                if r != robot and robot.distance_to(r) < NEIGHBOR_RADIUS and not r.is_anti
            ]
            if len(neighbors) >= 2 and np.random.rand() < STOP_PROB:
                robot.stopped = True

        robot.move()
        x_coords.append(robot.x)
        y_coords.append(robot.y)
        colors.append("black" if robot.stopped else "blue")

    # Anti-agent logic
    for anti in anti_agents:
        anti.move()
        for robot in robots:
            if robot.stopped:
                neighbors = [
                    r for r in robots
                    if r != robot and r.stopped and anti.distance_to(r) < NEIGHBOR_RADIUS
                ]
                if len(neighbors) + 1 >= LEAVE_THRESHOLD:
                    robot.stopped = False  # Leave command
                    print(f"Anti-agent at ({anti.x:.1f}, {anti.y:.1f}) told robot at ({robot.x:.1f}, {robot.y:.1f}) to leave")
                    break  # One robot per anti-agent per frame

        x_coords.append(anti.x)
        y_coords.append(anti.y)
        colors.append("red")

    num_clustered = sum(1 for r in robots if r.stopped)
    plt.scatter(x_coords, y_coords, c=colors)
    plt.xlim(0, WORLD_SIZE)
    plt.ylim(0, WORLD_SIZE)
    plt.title(f"Swarm Aggregation with Anti-Agents | Clustered Robots: {num_clustered}")
    plt.grid(True)

# Animate the simulation
fig = plt.figure(figsize=(6, 6))
ani = FuncAnimation(fig, update, frames=300, interval=100)
plt.show()