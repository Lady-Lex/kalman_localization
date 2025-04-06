import csv
import matplotlib.pyplot as plt
import numpy as np
import os

LOG_FILE = 'results/ekf_log_222834.csv'  # ← 修改为你的实际日志文件名

# === Prepare output folder ===
os.makedirs('results', exist_ok=True)

# === Read data ===
mu_xs, mu_ys, mu_thetas = [], [], []
z_xs, z_ys, z_thetas = [], [], []
gt_xs, gt_ys, gt_thetas = [], [], []
gt_vxs, gt_vys = [], []
kalman_gain_norms = []
times = []

with open(LOG_FILE, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        times.append(float(row['time']))
        mu_xs.append(float(row['mu_x']))
        mu_ys.append(float(row['mu_y']))
        mu_thetas.append(float(row['mu_theta']))

        z_xs.append(float(row['z_x']) if row['z_x'] else None)
        z_ys.append(float(row['z_y']) if row['z_y'] else None)
        z_thetas.append(float(row['z_theta']) if row.get('z_theta') else None)

        gt_xs.append(float(row['gt_x']) if row['gt_x'] else None)
        gt_ys.append(float(row['gt_y']) if row['gt_y'] else None)
        gt_thetas.append(float(row['gt_theta']) if row['gt_theta'] else None)

        gt_vxs.append(float(row['gt_vx']) if row['gt_vx'] else np.nan)
        gt_vys.append(float(row['gt_vy']) if row['gt_vy'] else np.nan)

        kalman_gain_norms.append(float(row['kalman_gain_norm']) if row.get('kalman_gain_norm') else np.nan)

# Convert to arrays
mu_xs = np.array(mu_xs)
mu_ys = np.array(mu_ys)
mu_thetas = np.array(mu_thetas)
z_thetas = np.array(z_thetas)
gt_xs = np.array(gt_xs)
gt_ys = np.array(gt_ys)
gt_thetas = np.array(gt_thetas)
gt_vxs = np.array(gt_vxs)
gt_vys = np.array(gt_vys)
kalman_gain_norms = np.array(kalman_gain_norms)
times = np.array(times)

# === Plot trajectory ===
plt.figure(figsize=(10, 6))
plt.plot(mu_xs, mu_ys, label='EKF Estimate', linewidth=2)
plt.scatter(z_xs, z_ys, label='Observations (z)', s=15, alpha=0.6)
z_xs_np = np.array([float(v) if v is not None else np.nan for v in z_xs])
z_ys_np = np.array([float(v) if v is not None else np.nan for v in z_ys])
plt.plot(z_xs_np, z_ys_np, label='Noisy Observation Trajectory', color='orange', linestyle=':', linewidth=1.5)
if np.any(~np.isnan(gt_xs)):
    plt.plot(gt_xs, gt_ys, label='Ground Truth', linestyle='--', linewidth=2)

plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('EKF Localization Trajectory')
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('results/trajectory.png')

# === Plot position error over time ===
plt.figure(figsize=(10, 5))
valid = ~np.isnan(gt_xs)
error_x = mu_xs[valid] - gt_xs[valid]
error_y = mu_ys[valid] - gt_ys[valid]
time_valid = times[valid]

plt.plot(time_valid, error_x, label='Error in x')
plt.plot(time_valid, error_y, label='Error in y')
plt.xlabel('Time [s]')
plt.ylabel('Estimation Error [m]')
plt.title('EKF Position Error Over Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig('results/position_error.png')

# === Plot velocity comparison ===
dt = np.diff(times)
vx_est = np.diff(mu_xs) / dt
vy_est = np.diff(mu_ys) / dt
t_mid = (times[1:] + times[:-1]) / 2

plt.figure(figsize=(10, 5))
plt.plot(t_mid, vx_est, label='EKF vx (estimated)')
plt.plot(t_mid, vy_est, label='EKF vy (estimated)')
plt.plot(times, gt_vxs, '--', label='Ground Truth vx')
plt.plot(times, gt_vys, '--', label='Ground Truth vy')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.title('Velocity Comparison (x and y directions)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig('results/velocity_comparison.png')

# === Plot theta error ===
def angle_diff(a, b):
    return np.arctan2(np.sin(a - b), np.cos(a - b))

valid_theta = ~np.isnan(gt_thetas)
theta_error = angle_diff(mu_thetas[valid_theta], gt_thetas[valid_theta])

plt.figure(figsize=(10, 5))
plt.plot(times[valid_theta], theta_error, label='Error in theta')
plt.xlabel('Time [s]')
plt.ylabel('Angular Error [rad]')
plt.title('Theta Estimation Error Over Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig('results/theta_error.png')

# === Plot Kalman Gain Norm over Time ===
valid_gain = ~np.isnan(kalman_gain_norms)
plt.figure(figsize=(10, 5))
plt.plot(times[valid_gain], kalman_gain_norms[valid_gain], label='Kalman Gain Frobenius Norm')
plt.xlabel('Time [s]')
plt.ylabel('||K|| (Frobenius)')
plt.title('Kalman Gain Magnitude Over Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig('results/kalman_gain_norm.png')

plt.show()