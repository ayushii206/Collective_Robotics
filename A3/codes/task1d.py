import random
import math
import numpy as np
import matplotlib.pyplot as plt
import os

needle_length = 0.7
space_btw_line = 1
trials = 1000
no_of_needles = list(range(1, 101, 1))

true_probability = (2*needle_length)/(space_btw_line*math.pi)
print("True Probability calculated ", true_probability)

outside_confidence_interval = []

for n in no_of_needles:
    outside_prob_range = 0
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
        #according to formula 
        error = 1.96 * math.sqrt((1/n)*P*(1-P))
        lower_val = P - error
        higher_val = P + error

        if true_probability < lower_val or true_probability > higher_val:
            outside_prob_range = outside_prob_range + 1

    ratio = outside_prob_range / trials
    outside_confidence_interval.append(ratio)
    
# Paths
output_folder = "A3/output/task1"  
os.makedirs(output_folder, exist_ok=True)

plt.plot(no_of_needles, outside_confidence_interval, label='Ratio Outside CI (Actual)', color='blue')
plt.axhline(0.05, linestyle='--', color='gray', label='Expected 5%')
plt.title("Ratio of Times True Probability is Outside 95% CI")
plt.xlabel("Number of Needle Throws per Experiment (n)")
plt.ylabel("Proportion Outside CI")
plt.legend()
plt.grid(True)
plt.tight_layout()
# Save the plot
filename = "task1d.png"
filepath = os.path.join(output_folder, filename)
plt.savefig(filepath)
plt.show()