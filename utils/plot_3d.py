import matplotlib.pyplot as plt
import numpy as np
import argparse

def load_trajectory(file_path):
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            values = [float(x) for x in line.strip().split()]
            # Check if file is KITTI format (12 columns) or simple XYZ (3 columns)
            if len(values) == 12:
                # Extract tx (idx 3), ty (idx 7), tz (idx 11)
                data.append([values[3], values[7], values[11]])
            elif len(values) >= 3:
                data.append(values[:3])
    return np.array(data)

def plot_trajectory(positions, title="3D Trajectory", output_file="trajectory_3d.png"):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot X axis = Data X (Left/Right)
    # Plot Y axis = Data Z (Forward/Back) 
    # Plot Z axis = Data Y (Up/Down)
    ax.plot(positions[:,0], positions[:,2], positions[:,1], label='Trajectory')
    
    # Set labels to match the data mapping
    ax.set_xlabel('X (Left/Right)')
    ax.set_ylabel('Z (Forward/Back)')
    ax.set_zlabel('Y (Up/Down)')
    
    # Invert Z axis label if Y data is "Down-positive" (common in CV)
    # ax.invert_zaxis() 
    plt.axis('equal')
    ax.set_title(title)
    ax.legend()
    plt.grid(True)
    plt.savefig(output_file)
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-i','--input', type=str, required=True, help='Trajectory text file')
    parser.add_argument('-o','--output', type=str, default="trajectory_3d.png", help="Output plot file")
    args = parser.parse_args()

    traj = load_trajectory(args.input)
    if traj.size == 0:
        print("Error: No trajectory data found.")
    else:
        plot_trajectory(traj, output_file=args.output)