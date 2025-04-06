import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
from kalman.ekf import ExtendedKalmanFilter


# === Configuration ===
CONFIG_PATH = 'config/ekf_config.yaml'

def test_ekf_convergence():
    ekf = ExtendedKalmanFilter(config_path=CONFIG_PATH)

    # Simulated ground truth trajectory (straight line)
    true_states = []
    measurements = []
    control_inputs = []

    x, y, theta = 0.0, 0.0, 0.0
    v, omega = 1.0, 0.0   # Constant forward motion
    dt = 0.1
    steps = 50

    for _ in range(steps):
        # Ground truth update
        x += v * dt * np.cos(theta) + np.random.normal(0, 0.05)
        y += v * dt * np.sin(theta) + np.random.normal(0, 0.05)
        theta += omega * dt + np.random.normal(0, 0.01)


        true_states.append([x, y, theta])
        control_inputs.append(np.array([[v], [omega]]))

        # Simulated noisy observation (x, y only)
        z = np.array([[x + np.random.normal(0, 0.2)],
                      [y + np.random.normal(0, 0.2)]])
        measurements.append(z)

    # EKF loop
    estimates = []
    for u, z in zip(control_inputs, measurements):
        ekf.predict(u, dt)
        ekf.update(z)
        estimates.append(ekf.get_state().flatten())

    # Final error check
    final_est = estimates[-1]
    final_true = true_states[-1]
    error = np.linalg.norm(np.array(final_est[:2]) - np.array(final_true[:2]))

    print("Final estimated position:", final_est[:2])
    print("True final position:", final_true[:2])
    print("Position error:", error)

    assert error < 1.0, "EKF did not converge well (position error too high)"


if __name__ == '__main__':
    test_ekf_convergence()