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

```bash
python task1a.py     # Task 1A - Locust simulation
python task1b.py     # Task 1B - Transition histogram
python task1c.py     # Task 1C - Probabilistic trajectory
python task2a.py     # Task 2A - Delay diff eqs
python task2b.py     # Task 2B - Homing + reset model
```

---

## 📊 Results and Observations

### 🐜 Task 1A: Left-going Locusts Over Time

![Task 1A](task1a.png)

- Locusts initially move randomly.
- Clusters start to form where most locusts move in the same direction.
- Directional consensus emerges gradually.
- Spontaneous switching introduces noise and drift.

---

### 🔁 Task 1B: Transition Histogram

![Task 1B](task1b.png)

- The histogram shows transitions between states Lₜ → Lₜ₊₁.
- Diagonal dominance indicates stability—most states remain near their value.
- Spread around the diagonal suggests stochastic switching.

---

### 🔄 Task 1C: Probabilistic Model vs Original Simulation

![Task 1C](task1c.png)

- The sampled trajectory from the transition matrix (orange) is smoother and stabilizes near a high value.
- The original simulation (blue) shows more erratic behavior due to stochastic influences.
- The reduced model captures long-term trends but misses short-term fluctuations.

---

### 📉 Task 2A: Delay Differential Equations

![Task 2A](task2a.png)

- `m(t)` decreases exponentially as pucks are found.
- `n_s(t)` remains mostly constant due to delay τₐ = 2.
- Early dynamics are shaped heavily by delay handling.

---

### 🧠 Task 2B: With and Without m(80) Reset

#### Normal run:

![Task 2B](task2b.png)

- `n_s` (searching robots) increases as others finish homing.
- `n_h` (homing) drops after τₕ as robots return.
- `m` (pucks) depletes steadily.

#### With m(80) = 0.5 reset:

![Task 2B Reset](task2b_m(80).png)

- A sharp transition occurs at t = 80 due to added pucks.
- `n_s` increases again as robots resume searching.
- Shows how resource reset affects robot dynamics.

---

## 📁 File Structure

```
├── task1a.py
├── task1b.py
├── task1c.py
├── task2a.py
├── task2b.py
├── task1a.png
├── task1b.png
├── task1c.png
├── task2a.png
├── task2b.png
├── task2b_m(80).png
├── ES4_collective_robotics.pdf
├── README.md
```

---

## 👥 Contributors

- [Ayushi Arora](https://github.com/ayushii206)
