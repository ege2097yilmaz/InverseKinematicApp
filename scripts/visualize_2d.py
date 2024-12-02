import numpy as np
import matplotlib.pyplot as plt

def plot_2d_projections(filename):
    # Load joint positions
    joint_positions = np.loadtxt(filename)

    # Extract X, Y, Z coordinates
    x, y, z = joint_positions[:, 0], joint_positions[:, 1], joint_positions[:, 2]

    # Plot on X-Y plane
    plt.figure(figsize=(8, 6))
    plt.plot(x, y, marker='o', label="Arm Projection (X-Y)")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Robot Arm Projection on X-Y Plane")
    plt.grid()
    plt.legend()
    plt.axis('equal')
    plt.show()

    # Plot on Z-Y plane
    plt.figure(figsize=(8, 6))
    plt.plot(z, y, marker='o', label="Arm Projection (Z-Y)")
    plt.xlabel("Z")
    plt.ylabel("Y")
    plt.title("Robot Arm Projection on Z-Y Plane")
    plt.grid()
    plt.legend()
    plt.axis('equal')
    plt.show()

    # Plot on Z-X plane
    plt.figure(figsize=(8, 6))
    plt.plot(z, x, marker='o', label="Arm Projection (Z-X)")
    plt.xlabel("Z")
    plt.ylabel("X")
    plt.title("Robot Arm Projection on Z-X Plane")
    plt.grid()
    plt.legend()
    plt.axis('equal')
    plt.show()

# Call the function with the file containing joint positions
plot_2d_projections("../datas/joint_positions_solution_1.txt")
