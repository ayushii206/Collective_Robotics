# Collective Robotics - Assignment 1

This repository contains code and outputs for Assignment 1 of the **Collective Robotics** course.  
It covers simulations and analysis tasks related to Poisson processes and synchronized flashing in fireflies.

## Requirements

1. Python 3.8+
2. Jupyter Notebook (Task 1)
3. Python Libraries:

    - numpy
    - matplotlib
    - scipy
    - tqdm

You can install the python packages using pip:

```bash
pip3 install matplotlib scipy numpy tqdm
```
---

## How to Run?

### Task 1: Scaling of a Data Center

**Objective:** Model a simple data center where incoming data follows a Poisson process, and investigate how different arrival rates and processing times affect the waiting list length through simulation and plotting.

**Steps:** Use the following commands to run the task:

```bash
cd A1/codes
jupyter notebook task1.ipynb
```
- Open the notebook and run all cells.

- Outputs (plots and graphs) will be saved in output/task1/.

### Task 2: Synchronization of a Swarm

**Objective:** Simulate a swarm of fireflies synchronizing their flashing cycles without a central clock, exploring how local neighbor interactions and vicinity distances influence synchronization efficiency.

**Steps:** Use the following commands to run the subtasks:

```bash
python3 A1/codes/task2a.py
```

```bash
python3 A1/codes/task2b.py
```

- Outputs (plots and graphs) will be saved in output/task2/.

## 👥 Contributors:
- [Ayushi Arora](https://github.com/ayushii206)
- [Kamran Ali](https://github.com/kamrankhowaja)