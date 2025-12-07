import numpy as np
import matplotlib.pyplot as plt
import pykitti
import os
import argparse

def load_estimated_trajectory(file_path):
    """
    Loads a trajectory file (KITTI format).
    Args:
        file_path: Path to the trajectory file
    Expected Format: 12 floats per line (r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz)
    Returns: Nx3 array [x, y, z]
    """
    xyz = []
    with open(file_path, 'r') as f:
        for line in f:
            val = list(map(float, line.strip().split()))
            # Extract translations tx (idx 3), ty (idx 7), tz (idx 11)
            xyz.append([val[3], val[7], val[11]])
    
    return np.array(xyz)

def load_ground_truth_trajectory(basedir, date, drive):
    """
    Loads raw KITTI data and converts IMU coordinates to Camera Frame.
    Args:
        basedir: Root directory of KITTI raw data
        date: Date string of the drive
        drive: Drive number string
    Returns: Nx3 array [x, y, z] where Z is forward, X is right, Y is down.
    """
    dataset = pykitti.raw(basedir, date, drive)
    
    # Get the initial IMU pose (World Origin)
    T_w_imu_0 = dataset.oxts[0].T_w_imu
    T_imu_0_w = np.linalg.inv(T_w_imu_0)

    gt_xyz_camera = []

    for oxt in dataset.oxts:
        # Get current global pose of IMU
        T_w_imu_i = oxt.T_w_imu
        
        # Calculate movement relative to starting IMU Frame
        T_rel = T_imu_0_w.dot(T_w_imu_i)
        
        # Extract translation in IMU frame and convert to Camera Frame
        x_imu = T_rel[0, 3] # Forward
        y_imu = T_rel[1, 3] # Left
        z_imu = T_rel[2, 3] # Up

        x_cam = -y_imu # Right
        y_cam = -z_imu # Down
        z_cam = x_imu  # Forward
        
        gt_xyz_camera.append([x_cam, y_cam, z_cam])

    return np.array(gt_xyz_camera)

def align_trajectories(est, gt):
    """
    Computes similarity transform (Umeyama) that aligns 'est' to 'gt'.
    Args:
        est: Estimated trajectory Nx3
        gt: Ground truth trajectory Nx3
    Returns est_aligned, scale, Rotation (3x3), translation (3,)
    """
    assert est.shape == gt.shape, "Trajectory shapes do not match"
    n = est.shape[0]

    mean_est = est.mean(axis=0)
    mean_gt  = gt.mean(axis=0)

    # Center the points
    est_c = est - mean_est
    gt_c  = gt - mean_gt

    # Covariance matrix
    cov_matrix = (gt_c.T @ est_c) / n

    # Singular Value Decomposition
    U, D, Vt = np.linalg.svd(cov_matrix)

    # Handles reflection output from svd
    S = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        S[2, 2] = -1

    # Compute rotation, scale, and translation
    R = U @ S @ Vt
    variance_est = np.sum(est_c ** 2) / n
    scale = np.sum(D * np.diag(S)) / variance_est
    t = mean_gt - scale * R @ mean_est

    # Apply transformation: s * (R @ Pose.T).T + t
    est_aligned = scale * (R @ est.T).T + t
    return est_aligned, scale, R, t

def plot_trajectory(gt, est, output_dir, output_file='trajectory_comparison.png'):
    """
    Plots the aligned trajectories in the X-Z plane (Bird's Eye View).
    Args:
        gt: Ground truth trajectory Nx3
        est: Estimated trajectory Nx3
        output_dir: Directory to save the plot
    """
    plt.figure(figsize=(10, 6))
    
    # Plot XZ plane (Bird's Eye View for KITTI Camera Frame)
    plt.plot(gt[:, 0], gt[:, 2], label='Ground Truth', color='orange', linewidth=2, linestyle='--')
    plt.plot(est[:, 0], est[:, 2], label='Estimated Pose', color='blue')
    
    plt.title('Trajectory Comparison with Umeyama Alignment')
    plt.xlabel('X (meters)')
    plt.ylabel('Z (meters)')
    # plt.axis('equal')
    plt.xlim(min(gt[:, 0].min(), est[:, 0].min()) - 1, max(gt[:, 0].max(), est[:, 0].max()) + 1)
    plt.ylim(min(gt[:, 2].min(), est[:, 2].min()) - 1, max(gt[:, 2].max(), est[:, 2].max()) + 1)
    plt.legend()
    plt.grid(True)
    
    output_path = os.path.join(output_dir, output_file)
    plt.savefig(output_path)
    print(f"Plot saved to {output_path}")

def main():
    parser = argparse.ArgumentParser(description="Compare VO trajectory with KITTI ground truth.")
    parser.add_argument('-i', '--input', type=str, required=True, help='Path to the estimated trajectory file')
    parser.add_argument('-k', '--kitti_root', type=str, required=True, help='KITTI raw data root directory')
    parser.add_argument('-dt','--date', type=str, required=True, help='Date of the KITTI drive')
    parser.add_argument('-dr','--drive', type=str, required=True, help='Drive number')
    parser.add_argument('-o', '--output', type=str, default='./results', help='Output directory for results')
    args = parser.parse_args()

    # Ensure output directory exists
    if not os.path.exists(args.output):
        os.makedirs(args.output)
    
    # Load Data
    est_traj = load_estimated_trajectory(args.input)
    gt_traj = load_ground_truth_trajectory(args.kitti_root, args.date, args.drive)
    
    # Align Data with umeyama method, using the minimum length
    min_len = min(len(est_traj), len(gt_traj))
    est_traj = est_traj[:min_len]
    gt_traj = gt_traj[:min_len]

    est_aligned, scale, R, t = align_trajectories(est_traj, gt_traj)
    print(f"Alignment Results:\nScale: {scale}\nRotation:\n{R}\nTranslation:\n{t}")

    # Calculate Error
    error = gt_traj - est_aligned
    rmse = np.sqrt(np.mean(np.sum(error**2, axis=1)))
    print(f"\nResults:\nAbsolute Trajectory Error (RMSE): {rmse:.4f} meters")

    # Plot
    output_file = f"{args.date}_{args.drive}_comparison.png"
    plot_trajectory(gt_traj, est_aligned, args.output, output_file)

if __name__ == "__main__":
    main()