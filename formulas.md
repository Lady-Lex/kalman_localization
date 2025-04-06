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