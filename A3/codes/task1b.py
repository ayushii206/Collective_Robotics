import random 
import time 
import math
import numpy as np
import matplotlib.pyplot as plt
import os

needle_length = 0.7
space_btw_line = 1
trials = 10000

no_of_needles = list(range(10, 1001, 10))

std_deviations = []

for n in no_of_needles:
    Probibility = []
    
    for i in range(trials):
        crossed_line_count = 0
        for K in range(n):
            
            d_btw_needle_line = random.uniform(0,space_btw_line/2)
            angle_btw_needle_line = random.uniform(0,math.pi/2)
        
            # print(d_btw_needle_line,angle_btw_needle_line)
        
            crossing_line = math.sin(angle_btw_needle_line) * (needle_length/2)
            # print(crossing_line)
            if crossing_line > d_btw_needle_line:
                crossed_line_count = crossed_line_count + 1
            else:
                continue
        P = crossed_line_count/n
        Probibility.append(P)
        # print(f"Probability of getting the needle crossed with quantity{n} is {Probibility}")
    Std_Dev = np.std(Probibility)
    std_deviations.append(Std_Dev)

# Paths
output_folder = "A3/output/task1"  
os.makedirs(output_folder, exist_ok=True)

plt.plot(no_of_needles, std_deviations, marker='o')
plt.title("Standard Deviation of Intersection Probability vs Number of Needles")
plt.xlabel("Number of Needles per Experiment (n)")
plt.ylabel("Standard Deviation of P")
plt.grid(True)
# Save the plot
filename = "task1b.png"
filepath = os.path.join(output_folder, filename)
plt.savefig(filepath)
plt.show()