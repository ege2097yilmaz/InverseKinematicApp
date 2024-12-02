import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Read results from the file
with open("../datas/results.txt", "r") as file:
    lines = file.readlines()

# Parse joint angles and link lengths
theta1, theta2, theta3 = map(float, lines[0].split())
l1, l2, l3 = map(float, lines[1].split())
targetX, targetY, targetZ = map(float, lines[2].split())

# Convert angles to radians for visualization
theta1 = theta1
theta2 = theta2
theta3 = theta3

# Calculate joint positions
x0, y0, z0 = 0, 0, 0  # Base of the robot

# Shoulder joint
x1 = l1 * np.cos(theta1)
y1 = l1 * np.sin(theta1)
z1 = 0

# Elbow joint
x2 = x1 + l2 * np.cos(theta1) * np.cos(theta2)
y2 = y1 + l2 * np.sin(theta1) * np.cos(theta2)
z2 = z1 + l2 * np.sin(theta2)

# Wrist joint (end effector)
x3 = x2 + l3 * np.cos(theta1) * np.cos(theta2 + theta3)
y3 = y2 + l3 * np.sin(theta1) * np.cos(theta2 + theta3)
z3 = z2 + l3 * np.sin(theta2 + theta3)

# Plot the robotic arm
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot links
ax.plot([x0, x1], [y0, y1], [z0, z1], 'ro-', label="Link 1")
ax.plot([x1, x2], [y1, y2], [z1, z2], 'go-', label="Link 2")
ax.plot([x2, x3], [y2, y3], [z2, z3], 'bo-', label="Link 3")

# Plot joints
ax.scatter([x0, x1, x2, x3], [y0, y1, y2, y3], [z0, z1, z2, z3], color='k', label="Joints")

# Plot target point
ax.scatter([targetX], [targetY], [targetZ], color='red', s=100, label="Target Point", marker='x')

# Set labels
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
ax.set_title("3D Robotic Arm Configuration")

# Set equal scaling for all axes
max_range = max(abs(targetX), abs(targetY), abs(targetZ), l1 + l2 + l3)
ax.set_xlim([-max_range, max_range])
ax.set_ylim([-max_range, max_range])
ax.set_zlim([-max_range, max_range])

ax.legend()
plt.show()
