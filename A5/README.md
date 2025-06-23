# Collective Robotics – Assignment 5

This repository contains the implementation and analysis for **Assignment 5** of the *Collective Robotics* course (Summer Semester 2025, Prof. Dr. Javad Ghofrani). The tasks involve implementing the urn model, analyzing global switching behavior in locust swarms, and simulating foraging behavior using robot swarms.
---

## 📌 Task Overview

### **Task 1: Urn Model for Locust Scenario**

This task investigates the average change in the number of left-goers (L) in a locust swarm simulation.  
- **Subtask A**: Simulate a swarm and record ΔL(L) over many runs  
- **Subtask B**: Fit the swarm urn model using feedback equations to the measured ΔL(L)  
- **Subtask C**: Plot fitted function and feedback probability to evaluate model accuracy

**📍 Code**: See `A5/codes/A5_task1_task2.ipynb`  

**📈 Output**: Plots of ΔL(L), fitted model, and feedback probability  

**📄 Results**:  

- ΔL(L) = 0 at stable points indicates potential equilibrium.
- Feedback peaks when the swarm is split evenly (s = 0.5) and drops when fully aligned.
- The model effectively captures self-organizing dynamics.

---

### **Task 2: Density-dependent Global Switching**

This task tests how often a swarm switches global direction depending on its size (N ∈ [20, 150]).

**Approach**:
- Track zones based on L:  
  - Zone A: L > 0.7N  
  - Zone B: 0.3N ≤ L ≤ 0.7N  
  - Zone C: L < 0.3N  
- A switch is detected if the swarm goes from A → B → C or C → B → A.

**📍 Code**: Continued in `A5_task1_task2.ipynb` 

**📈 Output**:  
- Plot: *Average switch time vs N*  
- Plot: *Number of switches vs N*

**📄 Observations**:  
- Smaller swarms switch direction more frequently.
- Larger swarms stabilize longer in a dominant direction.
- Mimics real locust swarm dynamics as observed in biology.

---

### **Task 3: Foraging Simulation (PyGame)**

Implements a 2D foraging simulation using simple robots equipped with proximity, bumper, and light sensors. Robots pick up objects and return them to a bright-lit home zone.

**Functionality**:
- Robots move using motor actuation.
- They detect collisions, objects, and home zone.
- Simulation runs for different swarm sizes: N ∈ {1, 2, ..., 10}

**📍 Code**: `A5/codes/task3.py`  

**🎥 Output Video**: `A5/output/task3/foraging_swarm_size_10.mov`  

**📁 Logs**: Saved in `A5/output/task3/simulation_logs/` 

**📄 Results**:  

- Performance (number of collected objects) increases with swarm size up to a point.
- Beyond certain N, interference and congestion reduce efficiency.
- Ideal swarm size balances coverage and coordination.

---

## ⚙️ Requirements

To run Task 3 simulation, install:

```bash
pip install pygame
```
Other tasks require:

```bash
pip install numpy matplotlib
```

---

## 📁 File Structure

```
A5/
├── codes/
│   ├── A5_task1_task2.ipynb     # Task 1 & 2 analysis and plots
│   ├── task3.py                 # Task 3: Foraging simulation 
├── output/
│   ├── task1/                   # Outputs for task1
│   ├── task2/                   # Outputs for task2
│   └── task3/                   # Outputs for task3
├── ColRob_5.pdf                 # Task sheet (Assignment 5)
└── README.md                    # This file
```

---

## 👥 Contributors

- [Ayushi Arora](https://github.com/ayushii206)
- [Kamran Ali](https://github.com/kamrankhowaja)

---