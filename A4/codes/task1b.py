import numpy as np
import matplotlib.pyplot as plt
import os

no_of_locust = 20
circum = 1
percept_range = 0.045
time_stamp = 500
speed = [0.001 , -0.001]
num_of_runs = 1000

transition_matrix = np.zeros((no_of_locust + 1, no_of_locust + 1), dtype=int)

for run in range(num_of_runs):
    locust_pos = np.random.uniform(0, circum, no_of_locust)
    locust_speed = np.random.choice(speed, no_of_locust)

    for ts in range(time_stamp):
        left_count = np.sum(locust_speed < 0)
        
        new_speed = locust_speed.copy()
        
        for n in range(no_of_locust):
            locust_n_pos = locust_pos[n]
            locust_n_speed = locust_speed[n]
            
            diff = np.abs(locust_pos - locust_n_pos)
            distance_from_locust_n = np.minimum(diff, circum - diff)
            # print(f"Locus no {n} and distance from locust is {distance_from_locust_n}")
            # print(f"Time step {ts} | Locust {n} | Distances: {distance_from_locust_n}")
    
            neighbor_mask = (distance_from_locust_n < percept_range) & (np.arange(no_of_locust) != n)
            neighbor_speeds = locust_speed[neighbor_mask]
    
            opposite_count = np.sum(np.sign(neighbor_speeds) != np.sign(locust_n_speed))
            # print(f"Current Locust {n} its direction {locust_n_speed} and opposite count is {opposite_count} ")
    
            if len(neighbor_speeds) > 0 and opposite_count > len(neighbor_speeds) / 2:
                new_speed[n] = -locust_n_speed
            elif np.random.rand() < 0.015:
                new_speed[n] = -locust_n_speed

        left_count_next = np.sum(new_speed < 0)
        transition_matrix[left_count][left_count_next] += 1

        locust_speed = new_speed
        locust_pos = (locust_pos + locust_speed) % circum

# Paths
output_folder = "A4/output/task1"  
os.makedirs(output_folder, exist_ok=True)

plt.figure(figsize=(8, 6))
plt.imshow(transition_matrix, origin='lower', cmap='viridis', aspect='auto')
plt.colorbar(label='Transition Count')
plt.title("Transition Histogram $L_t \\rightarrow L_{t+1}$")
plt.xlabel("$L_{t+1}$ (Left-going after update)")
plt.ylabel("$L_t$ (Left-going before update)")
plt.xticks(np.arange(0, no_of_locust + 1, 2))
plt.yticks(np.arange(0, no_of_locust + 1, 2))
plt.tight_layout()
# Save the figure
filename = "task1b.png"
filepath = os.path.join(output_folder, filename)
plt.savefig(filepath)
plt.show()
