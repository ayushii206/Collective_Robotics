# Collective Robotics – Assignment 4

This repository contains the implementation and analysis for **Assignment 4** of the *Collective Robotics* course (Summer Semester 2025, Prof. Dr. Javad Ghofrani). The tasks involve **swarm behavior simulation**, **dimension reduction**, and **delay differential equation modeling** for collective robotics systems.

---

## 📌 Task Overview

**Task A**: Simulate locust behavior in a 1D ring-world, where agents switch direction based on neighbors or random chance.

**Task B**: Reduce the simulation to a model using only the number of left-goers, aggregate transition data, and create a 2D histogram.

**Task C**: Estimate transition probabilities from (B), simulate a stochastic trajectory using these, and compare it to the original simulation.

**Task 2A & 2B**: Solve rate equations and delay differential equations to model swarm search, avoidance, and homing behaviors over time.

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

## 📊 Results and Observations

### Task 1A: Left-going Locusts Over Time

- Locusts initially move randomly.
- Clusters start to form where most locusts move in the same direction.
- Directional consensus emerges gradually.
- Spontaneous switching introduces noise and drift.

---

### Task 1B: Transition Histogram

- The histogram shows transitions between states Lₜ → Lₜ₊₁.
- Diagonal dominance indicates stability—most states remain near their value.
- Spread around the diagonal suggests stochastic switching.

### Task 1C: Probabilistic Model vs Original Simulation

- The sampled trajectory from the transition matrix (orange) is smoother and stabilizes near a high value.
- The original simulation (blue) shows more erratic behavior due to stochastic influences.
- The reduced model captures long-term trends but misses short-term fluctuations.

---

### Task 2A: Delay Differential Equations

- `m(t)` decreases exponentially as pucks are found.
- `n_s(t)` remains mostly constant due to delay τₐ = 2.
- Early dynamics are shaped heavily by delay handling.

---

### Task 2B: With and Without m(80) Reset

#### Normal run:

- `n_s` (searching robots) increases as others finish homing.
- `n_h` (homing) drops after τₕ as robots return.
- `m` (pucks) depletes steadily.

#### With m(80) = 0.5 reset:

- A sharp transition occurs at t = 80 due to added pucks.
- `n_s` increases again as robots resume searching.
- Shows how resource reset affects robot dynamics.

---

## 🧠 Notes

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
- [Kamran Ali](https://github.com/kamrankhowaja)

---
