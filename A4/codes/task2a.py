import numpy as np
import matplotlib.pyplot as plt
from ddeint import ddeint
import os

# Parameters
alpha_r = 0.6
alpha_p = 0.2
tau_a = 2

def history(t):
    return np.array([1.0, 1.0])  # n_s(t), m(t)

def model(Y, t):
    n_s, m = Y(t)
    n_s_tau = Y(t - tau_a)[0] if t - tau_a >= 0 else history(t - tau_a)[0]
    
    dn_s_dt = -alpha_r * n_s * (n_s + 1) + alpha_r * n_s_tau * (n_s_tau + 1)
    dm_dt = -alpha_p * n_s * m
    
    return np.array([dn_s_dt, dm_dt])

time_span = np.linspace(0, 50, 1000)

sol = ddeint(model, history, time_span)
print('dde_values= ', sol)

# Paths
output_folder = "A4/output/task2"  
os.makedirs(output_folder, exist_ok=True)

plt.plot(time_span, sol[:,0], label='n_s(t)', color='red')
plt.plot(time_span, sol[:,1], label='m(t)', color='blue')
plt.xlabel('Time t')
plt.ylabel('Values')
plt.title('DDE solution for n_s(t) and m(t)')
plt.legend()
plt.grid(True)
# Save the plot
filename = "task2a.png"
filepath = os.path.join(output_folder, filename)
plt.savefig(filepath)
plt.show()
