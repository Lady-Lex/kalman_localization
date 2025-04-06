import numpy as np
import yaml
from kalman.utils import (
    motion_model,
    jacobian_G,
    jacobian_B,
    observation_model,
    jacobian_H,
    wrap_angle,
)


class ExtendedKalmanFilter:
    def __init__(self, config_path: str):
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.dim = config['state']['dim']

        init = config['state']['initial_state']
        self.state = np.array([[init['x']], [init['y']], [init['theta']]])

        cov = config['state']['initial_covariance']
        self.Sigma = np.diag([cov['x'], cov['y'], cov['theta']])

        self.control_enabled = config['control']['enabled']
        self.control_dim = config['control']['dim']
        self.use_cmd_vel = config['control'].get('use_cmd_vel', True)

        q = config['process_noise']
        self.R = np.diag([q['q_x'], q['q_y'], q['q_theta']])

        self.Q_u = np.eye(self.control_dim) * 1e-4
        self.use_dynamic_process_noise = config.get('use_dynamic_process_noise', False)

        r = config['measurement_noise']
        self.Q = np.diag([r['r_x'], r['r_y'], r['r_theta']])
        
        self.K = np.zeros((self.dim, self.dim))  

        self.dt = config.get('delta_t', 0.1)

    def predict(self, u: np.ndarray, dt: float = None):
        dt = dt or self.dt
        G_t = jacobian_G(self.state, u, dt)
        B_t = jacobian_B(self.state, u, dt)

        self.state = motion_model(self.state, u, dt)
        self.state[2, 0] = wrap_angle(self.state[2, 0])

        if self.use_dynamic_process_noise:
            R_t = B_t @ self.Q_u @ B_t.T
        else:
            R_t = self.R

        self.Sigma = G_t @ self.Sigma @ G_t.T + R_t

    def update(self, z: np.ndarray):
        H_t = jacobian_H(self.state)
        z_hat = observation_model(self.state)

        S = H_t @ self.Sigma @ H_t.T + self.Q
        K = self.Sigma @ H_t.T @ np.linalg.inv(S)
        self.K = K

        innovation = z - z_hat
        innovation[2, 0] = wrap_angle(innovation[2, 0])  # Normalize angle difference

        self.state = self.state + K @ innovation
        self.state[2, 0] = wrap_angle(self.state[2, 0])

        I = np.eye(self.dim)
        self.Sigma = (I - K @ H_t) @ self.Sigma

    def set_state(self, state: np.ndarray):
        """
        Manually set the EKF internal state mu (mean).

        Args:
            state (np.ndarray): A 3x1 vector [x, y, theta].
        """
        assert state.shape == (3, 1), "State must be a 3x1 column vector"
        self.state = state.copy()
        print(f"[EKF] State manually set to x={state[0,0]:.2f}, y={state[1,0]:.2f}, theta={state[2,0]:.2f}")

    def get_state(self) -> np.ndarray:
        return self.state.copy()

    def get_covariance(self) -> np.ndarray:
        return self.Sigma.copy()
