# Collective Robotics – Assignment 4

This repository contains the implementation and analysis for **Assignment 4** of the *Collective Robotics* course (Summer Semester 2025, Prof. Dr. Javad Ghofrani). The tasks involve **swarm behavior simulation**, **dimension reduction**, and **delay differential equation modeling** for collective robotics systems.

---

## 📌 Task Overview

**Task A**: Simulate locust behavior in a 1D ring-world, where agents switch direction based on neighbors or random chance.

**Task B**: Reduce the simulation to a model using only the number of left-goers, aggregate transition data, and create a 2D histogram.

**Task C**: Estimate transition probabilities from (B), simulate a stochastic trajectory using these, and compare it to the original simulation.

**Task 2A–2B**: Solve rate equations and delay differential equations to model swarm search, avoidance, and homing behaviors over time.

---

## ⚙️ Requirements

Install required Python packages using:

```bash
pip install numpy matplotlib
```

---

## ▶️ How to Run

Each subtask can be executed directly via Python:

### Task 1: Locust Swarm Simulation and Model Reduction

- **Subtask A**: Locust swarm simulation with direction-switching logic  
  ```bash
  python task1a.py
  ```

- **Subtask B**: Transition histogram generation from 1000 runs  
  ```bash
  python task1b.py
  ```

- **Subtask C**: Probabilistic trajectory sampling based on transition matrix  
  ```bash
  python task1c.py
  ```

### Task 2: Rate Equations with Delays

- **Subtask A**: Numerical solution for delay differential equations (searching & avoidance)  
  ```bash
  python task2a.py
  ```

- **Subtask B**: Extended model with homing state and dynamic puck reset  
  ```bash
  python task2b.py
  ```

---

## 📈 Does the Code Achieve the Intended Task?

- ✅ Implements spontaneous switching and neighbor-based direction logic for locusts.
- ✅ Performs dimension reduction to derive state-transition dynamics based on left-goer count.
- ✅ Uses a large sample of simulations to approximate probabilistic transition models.
- ✅ Simulates rate equations using Euler integration, accounting for time delays.
- ✅ Successfully incorporates homing behavior and dynamic state transitions over time.

Plots are generated for all tasks, including:
- Number of left-goers over time (1a)
- Transition heatmaps (1b)
- Simulated L(t) trajectories (1c)
- Population curves for searchers, avoiders, and homers (2a, 2b)

---

## 🧠 Conceptual Notes

- Locusts are modeled in a periodic 1D space (`C=1`) with binary direction and spontaneous behavior.
- Model reduction is done by aggregating across trajectories into a 2D transition histogram.
- Rate equations are delay-based, requiring manual handling of early-time conditions.
- Homing behavior introduces a memory effect (fixed-duration τh), resetting agents back to search state.

---

## 📁 File Structure

```
├── task1a.py        # Locust swarm simulation
├── task1b.py        # Transition histogram aggregation
├── task1c.py        # Probabilistic model trajectory generation
├── task2a.py        # Delay differential equations (searching & avoidance)
├── task2b.py        # Extended model with homing behavior
├── README.md        # This file
├── ES4_collective_robotics.pdf  # Assignment description
```

---

## 👥 Contributors

- [Ayushi Arora](https://github.com/ayushii206)

---
