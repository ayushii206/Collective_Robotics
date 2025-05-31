import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Parameters
N = 10            # Number of robots
alpha_p = 0.01    # Pickup rate
tau_h = 15        # Homing time
T = 160           # Simulation time

# For delay, we need to store history
def history(t):
    return [N, 0, 1]

# Helper to get delayed values
def get_delayed(arr, t, tau, t_eval):
    t_delayed = t - tau
    if t_delayed < 0:
        return arr[0]
    else:
        idx = np.searchsorted(t_eval, t_delayed)
        return arr[idx]

# ODE system with delay handled by interpolation
def robot_model(t, y, t_eval, n_s_hist, m_hist):
    n_s, n_h, m = y

    # Get delayed values for n_s and m
    if t < tau_h:
        n_s_tau = N
        m_tau = 1
    else:
        n_s_tau = np.interp(t - tau_h, t_eval, n_s_hist)
        m_tau = np.interp(t - tau_h, t_eval, m_hist)

    dn_s = -alpha_p * n_s * m + alpha_p * n_s_tau * m_tau
    dn_h = alpha_p * n_s * m - alpha_p * n_s_tau * m_tau
    dm   = -alpha_p * n_s * m
    return [dn_s, dn_h, dm]

# Time points
t_eval = np.linspace(0, T, 1001)
dt = t_eval[1] - t_eval[0]

# Initial conditions
y0 = [N, 0, 1]
ys = np.zeros((len(t_eval), 3))
ys[0] = y0

# Step through time, updating with delay
for i in range(1, len(t_eval)):
    t = t_eval[i]
    n_s_hist = ys[:i, 0]
    n_h_hist = ys[:i, 1]
    m_hist   = ys[:i, 2]
    y_prev = ys[i-1]

    # Euler step
    dydt = robot_model(t, y_prev, t_eval[:i], n_s_hist, m_hist)
    ys[i] = y_prev + np.array(dydt) * dt

# Plot results
plt.plot(t_eval, ys[:,0], label='n_s (searching)')
plt.plot(t_eval, ys[:,1], label='n_h (homing)')
plt.plot(t_eval, ys[:,2], label='m (pucks)')
plt.xlabel('Time')
plt.ylabel('Population')
plt.legend()
plt.grid(True)
plt.title('Robot States and Pucks Over Time')
plt.show()

# Second calculation: reset m(80) = 0.5 at t=80
ys2 = ys.copy()
idx80 = np.searchsorted(t_eval, 80)
ys2[idx80:,2] = ys2[idx80:,2] * (0.5 / ys2[idx80,2])

plt.plot(t_eval, ys2[:,0], label='n_s (searching)')
plt.plot(t_eval, ys2[:,1], label='n_h (homing)')
plt.plot(t_eval, ys2[:,2], label='m (pucks, reset at t=80)')
plt.xlabel('Time')
plt.ylabel('Population')
plt.legend()
plt.grid(True)
plt.title('With m(80) Reset to 0.5')
plt.show()
