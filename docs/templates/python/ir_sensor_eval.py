# conda create --name pes-env python=3.11.4 numpy scipy matplotlib pyserial control ipykernel

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Distance Data
dist_cm = np.array([5,10,15,20,25,30,35,40])

dist_mV = np.array([2200,1150,730,510,400,300,170,90])

# Define a fit region: in the example we only want [7, 70]
ind_fit = (dist_cm >= 7.0) & (dist_cm <= 70.0)


# Model function: a / (x + b)
def model(x, a, b):
    return a / (x + b)


# Curve fitting
popt, pcov = curve_fit(model, dist_mV[ind_fit], dist_cm[ind_fit])
a_opt, b_opt = popt
print("   Fitted parameters:")
print(f"   a = {a_opt:.4f},  b = {b_opt:.4f}")

# Evaluate the model for plotting ---
fitted_cm = model(dist_mV, a_opt, b_opt)

# Plot (3 subplots)
fig, axs = plt.subplots(1, 3, figsize=(15, 5))

# 1) dist_cm vs. dist_mV
axs[0].plot(dist_cm, dist_mV, "x-")
axs[0].grid(True)
axs[0].set_xlabel("Distance (cm)")
axs[0].set_ylabel("Voltage (mV)")
axs[0].set_xlim([0, dist_cm.max()])
axs[0].set_ylim([0, dist_mV.max()])

# 2) dist_mV vs. dist_cm with fitted model (over the fit region)
axs[1].plot(dist_mV, dist_cm, "x-", label="Measured")
axs[1].plot(dist_mV[ind_fit], fitted_cm[ind_fit], "x-", label="Fitted Fcn.")
axs[1].grid(True)
axs[1].set_ylabel("Distance (cm)")
axs[1].set_xlabel("Voltage (mV)")
axs[1].set_xlim([0, dist_mV.max()])
axs[1].set_ylim([0, dist_cm.max()])
axs[1].legend()

# 3) dist_cm vs. fitted_cm
axs[2].plot(dist_cm, dist_cm, "x-")
axs[2].plot(dist_cm, fitted_cm, "x-")
axs[2].grid(True)
axs[2].set_xlabel("Distance (cm)")
axs[2].set_ylabel("Fit (cm)")
axs[2].set_xlim([0, dist_cm.max()])
axs[2].set_ylim([0, dist_cm.max()])

plt.tight_layout()
plt.show()
