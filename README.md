# Program Usage

## 📦 Dependencies Installation

This project requires **Python 3.8+**.  
You can create a virtual environment and install the dependencies using the following commands:

```bash
pip install -r requirements.txt
```

## 🚀 Run EKF in Real Time

Make sure ROS and `rosbridge_server` are running:

```bash
rosbag play ./bags/slam_test.bag
roslaunch rosbridge_server rosbridge_websocket.launch
```

Then run:

```bash
python3 scripts/run_ekf.py
```

## 🔗 Required ROS Topics

The EKF expects the following topics to be available:

- `/cmd_vel` (`geometry_msgs/Twist`): control input (linear & angular velocity)
- `/odom` (`nav_msgs/Odometry`): noisy **pose and velocity** estimation from onboard SLAM/localization.

  > 📌 In this project, we treat `/odom` as **pseudo ground truth** (GT), i.e., high-confidence reference.  
  > Then simulate measurement noise on top of it to obtain a noisy observation $\mathbf{z}_t$:

  ```python
  z = gt + np.random.multivariate_normal(mean=[0, 0, 0], cov=self.Q).reshape(3, 1)
  ```

  This reflects a more realistic sensor model during filtering while maintaining consistent testing.

- `/kalman_odom` (`geometry_msgs/PoseWithCovarianceStamped`): published filtered state estimate from EKF

---

## 📈 Visualize Results

To visualize EKF performance and compare with ground truth:

```bash
python3 scripts/res_plot.py
```

Generates the following plots (saved in `results/`):

- EKF trajectory vs. ground truth vs. noisy observations
- Velocity estimation vs. ground truth (x, y directions)
- Error over time (position & angle)
- Angular estimation error (theta)

Output files:

- `trajectory.png`
- `velocity_comparison.png`
- `position_error.png`
- `theta_error.png`

---

## 🧪 Evaluate Accuracy (Metrics)

To evaluate filter accuracy with RMSE and maximum error:

```bash
python3 scripts/metrics.py --file results/ekf_log_<timestamp>.csv
```

It outputs metrics for:

- $x$, $y$ — position
- $\theta$ — heading angle

Example:

```text
=== Kalman Filter Accuracy Metrics ===
File: results/ekf_log_XXXXXX.csv
RMSE [x, y, theta]: [0.035 0.041 0.052]
Max Absolute Error: [0.19 0.12 0.31]
```

---

## 📁 Log File Structure

All runs log to `results/ekf_log_<timestamp>.csv` with the following fields:

| Field      | Description                   |
| ---------- | ----------------------------- |
| `time`     | Timestamp                     |
| `mu_*`     | Estimated state               |
| `z_*`      | Noisy observation             |
| `gt_*`     | Ground truth from `/odom`     |
| `gt_vx/vy` | Ground truth linear velocity  |
| `gt_omega` | Ground truth angular velocity |

---

## ⚙️ Config Parameters (ekf_config.yaml)

Example configuration:

```yaml
state:
  dim: 3
  initial_state:
    x: 0.0
    y: 0.0
    theta: 0.0
  initial_covariance:
    x: 0.1
    y: 0.1
    theta: 0.1

control:
  enabled: true
  dim: 2
  use_cmd_vel: true

process_noise:
  q_x: 0.01
  q_y: 0.01
  q_theta: 0.01

measurement_noise:
  r_x: 0.2
  r_y: 0.2
  r_theta: 0.1

delta_t: 0.1
use_dynamic_process_noise: true
```

### Notes:

- `measurement_noise.r_*`: controls the noise level added to simulated observations
- `process_noise.q_*`: base motion noise for fixed $R_t$
- `use_dynamic_process_noise: true`: enables adaptive motion noise:

$$
R_t = B_t Q_u B_t^\top
$$

Where $Q_u$ reflects uncertainty in control input $\mathbf{u}_t$.

---
