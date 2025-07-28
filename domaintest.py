import numpy as np
import matplotlib.pyplot as plt

# Link lengths
l1 = 7.2 
l2 = 4.9

# Joint limits
theta1_lim = (0, np.pi)
theta2_lim = (-np.pi/2, np.pi/2)

# Generate a grid of theta1 and theta2
theta1_vals = np.linspace(theta1_lim[0], theta1_lim[1], 200)
theta2_vals = np.linspace(theta2_lim[0], theta2_lim[1], 200)

# Compute reachable (x, y) positions
x_vals = []
y_vals = []

for theta1 in theta1_vals:
    for theta2 in theta2_vals:
        x1 = l1 * np.cos(theta1)
        y1 = l1 * np.sin(theta1)
        x2 = x1 + l2 * np.cos(theta1 + theta2)
        y2 = y1 + l2 * np.sin(theta1 + theta2)
        
        x_vals.append(x2)
        y_vals.append(y2)

# Plot the reachable workspace
plt.figure(figsize=(6,6))
plt.scatter(x_vals, y_vals, s=1, color='blue')
plt.title("Reachable Workspace of 2DOF Arm with Limited Joint Angles")
plt.xlabel("x")
plt.ylabel("y")
plt.axis("equal")
plt.grid(True)
plt.show()