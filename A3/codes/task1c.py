import random
import math
import numpy as np
import matplotlib.pyplot as plt
import os

needle_length = 0.7
space_btw_line = 1
trials = 1000
no_of_needles = list(range(1, 101, 1))

avg_prob = []
lower_Prob = []
higher_Prob = []

for n in no_of_needles:
    Probabilities = []
    
    for i in range(trials):
        crossed_line_count = 0
        for K in range(n):
            d = random.uniform(0, space_btw_line / 2)
            angle = random.uniform(0, math.pi / 2)
            crossing_line = math.sin(angle) * (needle_length / 2)
            if crossing_line > d:
                crossed_line_count += 1

        P_hat = crossed_line_count / n
        Probabilities.append(P_hat)

    avg_p = np.mean(Probabilities)
    avg_prob.append(avg_p)

    error = 1.96 * math.sqrt((avg_p * (1 - avg_p)) / n)
    lower_Prob.append(avg_p - error)
    higher_Prob.append(avg_p + error)

# Paths
output_folder = "A3/output/task1"  
os.makedirs(output_folder, exist_ok=True)

plt.plot(no_of_needles, avg_prob, label='Average P̂', color='blue')
plt.fill_between(no_of_needles, lower_Prob, higher_Prob, color='lightblue', alpha=0.5, label='95% CI')
plt.title("Estimated Intersection Probability and 95% Confidence Interval")
plt.xlabel("Number of Needles per Experiment (n)")
plt.ylabel("Estimated Probability (P̂)")
plt.legend()
plt.grid(True)
plt.tight_layout()
# Save the plot
filename = "task1c.png"
filepath = os.path.join(output_folder, filename)
plt.savefig(filepath)
plt.show()