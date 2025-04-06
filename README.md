# Extended Kalman Filter (EKF) — Full Mathematical Description

## 1. True State (Ground Truth)

$$
\mathbf{x}_t = 
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
\in \mathbb{R}^3
$$

This represents the actual (unobservable) system state.

---

## 2. State Estimate (Mean and Covariance)

We assume a Gaussian belief over the state:

$$
\mathbf{x}_t \sim \mathcal{N}(\boldsymbol{\mu}_t, \Sigma_t)
$$

Where:

- $\boldsymbol{\mu}_t \in \mathbb{R}^3$: estimated state mean  
- $\Sigma_t \in \mathbb{R}^{3 \times 3}$: covariance of the estimate

---

## 3. Control Input

$$
\mathbf{u}_t = 
\begin{bmatrix}
v \\
\omega
\end{bmatrix}
\in \mathbb{R}^2
$$

---

## 4. Motion Model (Nonlinear)

The robot follows a nonlinear motion model:

$$
\bar{\boldsymbol{\mu}}_t = g(\boldsymbol{\mu}_{t-1}, \mathbf{u}_t) =
\begin{bmatrix}
x + v \cdot \Delta t \cdot \cos(\theta) \\
y + v \cdot \Delta t \cdot \sin(\theta) \\
\theta + \omega \cdot \Delta t
\end{bmatrix}
$$

In reality, the motion is subject to Gaussian noise:

$$
\mathbf{x}_t = g(\mathbf{x}_{t-1}, \mathbf{u}_t) + \mathbf{w}_t, \quad \mathbf{w}_t \sim \mathcal{N}(\mathbf{0}, R_t)
$$

With Jacobian (w.r.t. state):

$$
G_t = \left. \frac{\partial g}{\partial \boldsymbol{\mu}} \right|_{\boldsymbol{\mu}_{t-1}, \mathbf{u}_t} =
\begin{bmatrix}
1 & 0 & -v \cdot \Delta t \cdot \sin(\theta) \\
0 & 1 &  v \cdot \Delta t \cdot \cos(\theta) \\
0 & 0 & 1
\end{bmatrix}
$$

---

## 5. Observation Model

Assuming we can observe position and orientation $(x, y, \theta)$:

$$
\hat{\mathbf{z}}_t = h(\bar{\boldsymbol{\mu}}_t) =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
$$

And the actual observation is:

$$
\mathbf{z}_t = h(\mathbf{x}_t) + \mathbf{v}_t, \quad \mathbf{v}_t \sim \mathcal{N}(\mathbf{0}, Q_t)
$$

With observation Jacobian:

$$
H_t = \left. \frac{\partial h}{\partial \mathbf{x}} \right|_{\bar{\boldsymbol{\mu}}_t} =
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

---

## 6. Covariances

- $R_t$: process noise covariance matrix (uncertainty in motion model)
- $Q_t$: measurement noise covariance matrix (sensor uncertainty)

---

## 7. EKF Algorithm (with Formula Blocks)

$$
\begin{aligned}
\textbf{Prediction step:} \\
\bar{\boldsymbol{\mu}}_t &= g(\boldsymbol{\mu}_{t-1}, \mathbf{u}_t) \\
\bar{\Sigma}_t &= G_t \, \Sigma_{t-1} \, G_t^\top + R_t \\[1.2em]

\textbf{Correction step:} \\
K_t &= \bar{\Sigma}_t \, H_t^\top \left( H_t \, \bar{\Sigma}_t \, H_t^\top + Q_t \right)^{-1} \\
\boldsymbol{\mu}_t &= \bar{\boldsymbol{\mu}}_t + K_t \left( \mathbf{z}_t - h(\bar{\boldsymbol{\mu}}_t) \right) \\
\Sigma_t &= (I - K_t H_t) \, \bar{\Sigma}_t
\end{aligned}
$$

---

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
- $	heta$ — heading angle

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
