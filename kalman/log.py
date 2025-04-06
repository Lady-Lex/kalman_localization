import csv
import os
import numpy as np
from typing import Optional

class ResultsLogger:
    def __init__(self, filepath: str):
        self.filepath = filepath
        self.fields = [
            'time',
            'mu_x', 'mu_y', 'mu_theta',
            'z_x', 'z_y', 'z_theta',
            'gt_x', 'gt_y', 'gt_theta',
            'gt_vx', 'gt_vy', 'gt_omega',
            'kalman_gain_norm'
        ]
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        self.file = open(filepath, mode='w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=self.fields)
        self.writer.writeheader()

    def log(self, timestamp: float,
            mu: np.ndarray,
            z: Optional[np.ndarray] = None,
            gt: Optional[np.ndarray] = None,
            gt_vel: Optional[np.ndarray] = None,
            kalman_gain_norm: Optional[float] = None):

        row = {
            'time': timestamp,
            'mu_x': mu[0, 0],
            'mu_y': mu[1, 0],
            'mu_theta': mu[2, 0],
            'z_x': z[0, 0] if z is not None else '',
            'z_y': z[1, 0] if z is not None else '',
            'z_theta': z[2, 0] if z is not None else '',
            'gt_x': gt[0, 0] if gt is not None else '',
            'gt_y': gt[1, 0] if gt is not None else '',
            'gt_theta': gt[2, 0] if gt is not None else '',
            'gt_vx': gt_vel[0, 0] if gt_vel is not None else '',
            'gt_vy': gt_vel[1, 0] if gt_vel is not None else '',
            'gt_omega': gt_vel[2, 0] if gt_vel is not None else '',
            'kalman_gain_norm': kalman_gain_norm if kalman_gain_norm is not None else ''
        }
        self.writer.writerow(row)

    def close(self):
        self.file.close()