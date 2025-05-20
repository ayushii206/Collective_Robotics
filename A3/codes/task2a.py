class Robot:
    def __init__(self, is_anti=False):
        self.x = random.randint(0, GRID_SIZE-1)
        self.y = random.randint(0, GRID_SIZE-1)
        self.carrying = False
        self.is_anti = is_anti

    def move(self):
        self.x = (self.x + random.choice([-1, 0, 1])) % GRID_SIZE
        self.y = (self.y + random.choice([-1, 0, 1])) % GRID_SIZE

    def act(self, grid):
        density = compute_density(grid, self.x, self.y)
        if not self.carrying:
            if grid[self.x][self.y] == 1:
                prob = 1 - P_pick(density) if self.is_anti else P_pick(density)
                if random.random() < prob:
                    self.carrying = True
                    grid[self.x][self.y] = 0
        else:
            if grid[self.x][self.y] == 0:
                prob = 1 - P_drop(density) if self.is_anti else P_drop(density)
                if random.random() < prob:
                    self.carrying = False
                    grid[self.x][self.y] = 1

import pygame
import random
import math
import numpy as np
import matplotlib.pyplot as plt
import os

GRID_SIZE = 100           
CELL_SIZE = 6            
NUM_ROBOTS =50           
NUM_OBJECTS = 200         
TICKS = 1000              
ANTI_AGENT_PERCENTAGES = [0, 2, 5, 10, 15]  

def compute_density(grid, x, y, radius=3):
    count = 0
    for dx in range(-radius, radius+1):
        for dy in range(-radius, radius+1):
            if 0 <= x+dx < GRID_SIZE and 0 <= y+dy < GRID_SIZE:
                if grid[x+dx][y+dy] == 1:
                    count += 1
    return count / ((2*radius+1)**2)

def P_pick(density):
    return max(0.1, 1 - density)

def P_drop(density):
    return min(0.9, density)


def run_simulation(anti_percentage, run_index=0):
    pygame.init()
    screen = pygame.display.set_mode((GRID_SIZE*CELL_SIZE, GRID_SIZE*CELL_SIZE))
    pygame.display.set_caption("Object Clustering with Anti-Agents")

    # Initialize grid and robots
    grid = [[0]*GRID_SIZE for _ in range(GRID_SIZE)]
    for _ in range(NUM_OBJECTS):
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        grid[x][y] = 1

    num_anti = int(NUM_ROBOTS * anti_percentage / 100)
    robots = [Robot(is_anti=(i < num_anti)) for i in range(NUM_ROBOTS)]

    for tick in range(TICKS):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        screen.fill((255, 255, 255))

        # Draw objects
        for x in range(GRID_SIZE):
            for y in range(GRID_SIZE):
                if grid[x][y] == 1:
                    pygame.draw.rect(screen, (0, 0, 0), (x*CELL_SIZE, y*CELL_SIZE, CELL_SIZE, CELL_SIZE))

        # Robots move and act
        for robot in robots:
            robot.move()
            robot.act(grid)
            color = (255, 0, 0) if robot.is_anti else (0, 0, 255)
            pygame.draw.circle(screen, color, (robot.x*CELL_SIZE+CELL_SIZE//2, robot.y*CELL_SIZE+CELL_SIZE//2), CELL_SIZE//2)

        pygame.display.flip()

    #pygame.image.save(screen, f"screenshot_{anti_percentage}percent_run{run_index}.png")
    pygame.quit()


    cluster_sizes = []
    visited = np.zeros((GRID_SIZE, GRID_SIZE))

    def dfs(x, y):
        if not (0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE):
            return 0
        if grid[x][y] == 0 or visited[x][y]:
            return 0
        visited[x][y] = 1
        size = 1
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                size += dfs(x + dx, y + dy)
        return size

    for x in range(GRID_SIZE):
        for y in range(GRID_SIZE):
            if grid[x][y] == 1 and not visited[x][y]:
                cluster_sizes.append(dfs(x, y))

    return max(cluster_sizes) if cluster_sizes else 0

results = []

for percent in ANTI_AGENT_PERCENTAGES:
    print(f"Running simulation with {percent}% anti-agents...")
    repetitions = 5
    scores = [run_simulation(percent, run_index=i) for i in range(repetitions)]
    avg_score = np.mean(scores)
    results.append(avg_score)

# Paths
output_folder = "A3/output/task2"  
os.makedirs(output_folder, exist_ok=True)

# Plot results
plt.figure(figsize=(8, 5))
plt.plot(ANTI_AGENT_PERCENTAGES, results, marker='o')
plt.title("Clustering Performance vs Anti-Agent Percentage")
plt.xlabel("Anti-Agent Percentage (%)")
plt.ylabel("Max Cluster Size (Average over runs)")
plt.grid(True)
plt.tight_layout()
# Save the plot
filename = "task2a.png"
filepath = os.path.join(output_folder, filename)
plt.savefig(filepath)
plt.show()
