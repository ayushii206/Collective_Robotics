# Collective Robotics - Assignment 3

This repository contains code and outputs for Assignment 3 of the **Collective Robotics** course. 

## Requirements

1. Python 3.8+
2. Jupyter Notebook (Task 1)
3. Python Libraries:

    - numpy
    - matplotlib
    - random
    - time

You can install the python packages using pip:

```bash
pip3 install matplotlib numpy time
```
---

## How to Run?

### Task 1: Buﬀon’s needle

**Objective:** To simulate Buffon's Needle experiment computationally to estimate the value of π and to analyze the statistical properties of the intersection probability using different sample sizes, standard deviation, confidence intervals, and error rates.

**Steps:** 

- Use the following commands to run each subtask.
- Outputs (plots and graphs) of  each subtask will be saved in output/task1/

#### Subtask A: Estimate Intersection Probability and Approximate π

**Objective:** Simulate the simplified Buffon’s needle experiment and compute the probability that the needle intersects a line.

```bash
python3 python3 /A3/codes/task1a.py 
```

#### Subtask B: Analyze Standard Deviation of Intersection Probabilities

**Objective:** Run 10,000 experiments for varying trial counts n ∈ {10,20,...,1000}, compute the standard deviation of intersection probabilities, and visualize how variability reduces as the number of trials increases.

```bash
python3 python3 /A3/codes/task1b.py 
```

#### Subtask C: Plot Probability and 95% Confidence Interval (up to n = 100)

**Objective:** For a maximum of 100 needle drops, simulate multiple experiments and plot the measured intersection probability along with the Binomial 95% confidence interval, to observe uncertainty behavior in early trials.

```bash
python3 python3 /A3/codes/task1c.py 
```

#### Subtask D:  Check Confidence Interval Coverage

**Objective:** Determine how often the true intersection probability falls outside the estimated 95% confidence intervals, and plot this error rate over different trial counts to assess how reliable the confidence intervals are as n increases.

```bash
python3 python3 /A3/codes/task1d.py 
```

### Task 2: Anti-agents in swarm aggregation

**Objective:** To explore the counterintuitive effects of anti-agents in swarm behaviors (either object clustering or robot aggregation) and to quantitatively evaluate their impact on swarm performance by simulating different anti-agent ratios.

**Steps:** 

- Use the following commands to run each subtask.
- Outputs (plots and graphs) of  each subtask will be saved in output/task2/

#### Subtask A: Object Clustering with Anti-Agents

**Objective:** Simulate the standard object clustering behavior using probabilistic pick-up/drop logic. Introduce anti-agents that invert the decision logic (i.e., using 1−P _pick and 1−P_drop), and evaluate how they affect the formation and quality of object clusters. Optionally, make these probabilities depend on local object density.

```bash
python3 python3 /A3/codes/task2a.py 
```

#### Subtask B: Robot Aggregation with Anti-Agents

**Objective:** Simulate robot-only aggregation (no objects), where robots cluster together. Introduce anti-agents that actively disturb clusters by instructing robots to leave clusters based on the local density. Study how this dynamic affects the overall swarm's ability to aggregate and re-aggregate.

```bash
python3 python3 /A3/codes/task2b.py 
```

#### Subtask C: Vary Anti-Agent Percentage and Evaluate Performance

**Objective:** Run repeated experiments with different ratios of anti-agents (preferably small fractions like 0%, 1%, 3%, etc.) and quantify the aggregation performance—e.g., via maximum cluster size or cluster count after a fixed duration. Determine if and when anti-agents enhance or deteriorate swarm coordination, and reflect on the difficulty in replicating earlier findings.

```bash
python3 python3 /A3/codes/task2c.py 
```

###  Observations:

- **Task2a:** The clustering performance improves with the addition of a small percentage of anti-agents, peaking at 5%. However, performance drops sharply at 10%, before slightly recovering at 15%. This suggests that a moderate number of anti-agents can enhance clustering, but too many may disrupt the process.
- **Task2b:** In the simulation, robots initially move randomly and begin to form clusters based on local neighbor density. Anti-agents actively disrupt overly large clusters (3  robots inthis  casse) by prompting robots to leave, leading to dynamic, self-regulating aggregation. This results in a balance between cluster formation and dispersal over time.
- **Task2c:** The plot shows that swarm clustering improves as anti-agent percentage increases, peaking around 13%. Beyond this point, performance declines, indicating that moderate disruption enhances aggregation, but too many anti-agents become counterproductive. This forms a clear inverted-U trend, validating the expected non-linear effect.

## 👥 Contributors:
- [Ayushi Arora](https://github.com/ayushii206)
- [Kamran Ali](https://github.com/kamrankhowaja)