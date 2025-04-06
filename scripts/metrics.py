import csv
import numpy as np
import argparse


def angle_diff(a, b):
    return np.arctan2(np.sin(a - b), np.cos(a - b))


def compute_metrics(log_file):
    mu_list = []
    gt_list = []

    with open(log_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['gt_x'] and row['gt_y'] and row['gt_theta']:
                mu = [float(row['mu_x']), float(row['mu_y']), float(row['mu_theta'])]
                gt = [float(row['gt_x']), float(row['gt_y']), float(row['gt_theta'])]
                mu_list.append(mu)
                gt_list.append(gt)

    mu_arr = np.array(mu_list)
    gt_arr = np.array(gt_list)
    error = mu_arr - gt_arr
    error[:, 2] = angle_diff(mu_arr[:, 2], gt_arr[:, 2])  # 👈 只对 theta 做包裹处理

    rmse = np.sqrt(np.mean(error ** 2, axis=0))
    max_error = np.max(np.abs(error), axis=0)

    print("=== Kalman Filter Accuracy Metrics ===")
    print(f"File: {log_file}")
    print(f"RMSE [x, y, theta]: {rmse}")
    print(f"Max Absolute Error: {max_error}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', type=str, required=True, help='Path to EKF log CSV file')
    args = parser.parse_args()

    compute_metrics(args.file)
