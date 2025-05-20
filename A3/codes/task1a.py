import random 
import time 
import math

needle_length = 0.7
space_btw_line = 1
trials = 100
crossed_line_count = 0

for i in range(trials):
    d_btw_needle_line = random.uniform(0,space_btw_line/2)
    angle_btw_needle_line = random.uniform(0,math.pi/2)

    # print(d_btw_needle_line,angle_btw_needle_line)

    crossing_line = math.sin(angle_btw_needle_line) * (needle_length/2)
    # print(crossing_line)
    if crossing_line > d_btw_needle_line:
        crossed_line_count = crossed_line_count + 1
    else:
        continue

Probibility = crossed_line_count/trials
print(f"Probability of getting the needle crossed is {Probibility}")

Pi_estimate = (2*needle_length) / (space_btw_line*Probibility)
print(f"Pi estimated {Pi_estimate} vs Pi Actual 3.142")