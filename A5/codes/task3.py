"""
Simulation of a foraging task using Pygame.

This script creates an animated 2D simulation where multiple robots forage objects 
from a random environment and transport them back to a home location. Each robot 
is equipped with simple behaviors for object detection, pickup, and return-to-home logic.
Sensor data such as light, proximity, and bumper-like behaviors are simulated to enable 
local decision-making.

Key Features:
- Pygame animation of robots, objects, and home base.
- Logging of robot sensor states and behavior at each timestep.
- End-of-simulation plot of robot activities and performance metrics.
- CSV output of all robot sensor readings and actions for post-analysis.
"""

import os
import csv
import pygame
import math
import random
import matplotlib.pyplot as plt

WIDTH, HEIGHT = 800, 600
FPS = 30
ROBOT_RADIUS = 10
OBJECT_SIZE = 8
HOME_RADIUS = 60
SIMULATION_TIME = 20  # seconds
HOME_POS = (HOME_RADIUS, HOME_RADIUS)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 150, 255)
RED = (255, 50, 50)

log_dir = "A5/output/task3/simulation_logs"
os.makedirs(log_dir, exist_ok=True)

class Object:
    def __init__(self, x, y):
        self.pos = [x, y]
        self.collected = False

class Robot:
    def __init__(self, idx, robots):
        self.id = idx
        self.x = random.randint(100, WIDTH - 100)
        self.y = random.randint(100, HEIGHT - 100)
        self.angle = random.uniform(0, 2 * math.pi)
        self.speed = 2
        self.has_object = False
        self.obj = None
        self.robots = robots
        self.log = []

    def move(self):
        if self.has_object:
            dx = HOME_POS[0] - self.x
            dy = HOME_POS[1] - self.y
            self.angle = math.atan2(dy, dx) + random.uniform(-0.2, 0.2)
        else:
            if random.random() < 0.05:
                self.angle += random.uniform(-0.5, 0.5)

        self.x += self.speed * math.cos(self.angle)
        self.y += self.speed * math.sin(self.angle)

        if self.has_object and self.obj:
            self.obj.pos[0] = self.x
            self.obj.pos[1] = self.y

        if self.x < ROBOT_RADIUS or self.x > WIDTH - ROBOT_RADIUS:
            self.angle = math.pi - self.angle
        if self.y < ROBOT_RADIUS or self.y > HEIGHT - ROBOT_RADIUS:
            self.angle = -self.angle

    def check_sensors_and_log(self, objects):
        proximity = any(
            math.hypot(self.x - r.x, self.y - r.y) < 2 * ROBOT_RADIUS and r != self
            for r in self.robots
        )
        bumper = any(
            abs(self.x - obj.pos[0]) < OBJECT_SIZE and abs(self.y - obj.pos[1]) < OBJECT_SIZE and not obj.collected
            for obj in objects
        )
        light_left = 1 / (math.hypot(HOME_POS[0] - self.x, HOME_POS[1] - self.y) + 1)
        light_right = light_left  # symmetric light
        boundary = (
            self.x <= ROBOT_RADIUS or self.x >= WIDTH - ROBOT_RADIUS or
            self.y <= ROBOT_RADIUS or self.y >= HEIGHT - ROBOT_RADIUS
        )
        transporting = self.has_object
        home_zone = math.hypot(self.x - HOME_POS[0], self.y - HOME_POS[1]) < HOME_RADIUS
        error = False

        self.log.append([
            self.id, proximity, bumper, light_left, light_right,
            boundary, transporting, home_zone, error
        ])

    def check_object_pickup(self, objects):
        if self.has_object:
            return
        for obj in objects:
            if not obj.collected:
                dx = self.x - obj.pos[0]
                dy = self.y - obj.pos[1]
                if abs(dx) < OBJECT_SIZE and abs(dy) < OBJECT_SIZE:
                    self.has_object = True
                    self.obj = obj
                    obj.collected = True
                    return

    def drop_object(self):
        if self.has_object and math.hypot(self.x - HOME_POS[0], self.y - HOME_POS[1]) < HOME_RADIUS:
            self.has_object = False
            self.obj = None

def run_simulation_with_logging(swarm_size=5, object_count=15, round_num=1):
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(f"Foraging Simulation - {swarm_size} Robots")
    clock = pygame.time.Clock()

    robots = []
    for i in range(swarm_size):
        robots.append(Robot(i, robots))

    objects = [Object(random.randint(150, WIDTH - 50), random.randint(150, HEIGHT - 50)) for _ in range(object_count)]

    start_ticks = pygame.time.get_ticks()
    running = True

    while running:
        screen.fill(WHITE)
        pygame.draw.circle(screen, YELLOW, HOME_POS, HOME_RADIUS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for robot in robots:
            robot.move()
            robot.check_object_pickup(objects)
            robot.drop_object()
            robot.check_sensors_and_log(objects)
            pygame.draw.circle(screen, RED if robot.has_object else BLUE, (int(robot.x), int(robot.y)), ROBOT_RADIUS)

        for obj in objects:
            if not obj.collected:
                pygame.draw.rect(screen, BLACK, (*obj.pos, OBJECT_SIZE, OBJECT_SIZE))

        pygame.display.flip()
        clock.tick(FPS)

        if (pygame.time.get_ticks() - start_ticks) / 1000 > SIMULATION_TIME:
            running = False

    pygame.quit()

    # Save logs
    for robot in robots:
        filename = f"robot_{robot.id}_swarm{swarm_size}_round{round_num}.csv"
        with open(os.path.join(log_dir, filename), "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "robot_id", "proximity", "bumper", "light_left", "light_right",
                "boundary", "transporting", "home_zone", "error"
            ])
            writer.writerows(robot.log)

    collected = sum(
        1 for obj in objects if obj.collected and math.hypot(obj.pos[0] - HOME_POS[0], obj.pos[1] - HOME_POS[1]) < HOME_RADIUS
    )
    return collected

# variable swarm size simulation
swarm_sizes = list(range(1, 11))
results = []

for i, swarm_size in enumerate(swarm_sizes):
    print(f"Running simulation for swarm size: {swarm_size}")
    collected = run_simulation_with_logging(swarm_size=swarm_size, object_count=20, round_num=i+1)
    results.append(collected)

# Plot
plt.figure(figsize=(8, 5))
plt.plot(swarm_sizes, results, marker='o', color='orange')
plt.title("Swarm Performance vs Swarm Size")
plt.xlabel("Swarm Size (Number of Robots)")
plt.ylabel("Objects Collected in Fixed Time")
plt.grid(True)
plt.xticks(swarm_sizes)
plt.tight_layout()
plt.show()
