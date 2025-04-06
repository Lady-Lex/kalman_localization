import numpy as np


def wrap_angle(theta):
    """
    Normalize an angle to [-pi, pi].
    """
    return (theta + np.pi) % (2 * np.pi) - np.pi

def motion_model(mu, u, dt):
    """
    Nonlinear motion model g(u, mu) for differential drive robot.

    Args:
        mu: state estimate at time t-1 (3x1) -> [x, y, theta]
        u: control input at time t (2x1) -> [v, omega]
        dt: timestep

    Returns:
        mu_bar: predicted state (3x1)
    """
    x, y, theta = mu.flatten()
    v, omega = u.flatten()

    x_new = x + v * dt * np.cos(theta)
    y_new = y + v * dt * np.sin(theta)
    theta_new = wrap_angle(theta + omega * dt)

    return np.array([[x_new], [y_new], [theta_new]])

def jacobian_G(mu, u, dt):
    """
    Jacobian G_t = df/dmu of the motion model w.r.t state.
    """
    _, _, theta = mu.flatten()
    v, _ = u.flatten()

    G = np.array([
        [1, 0, -v * dt * np.sin(theta)],
        [0, 1,  v * dt * np.cos(theta)],
        [0, 0, 1]
    ])
    return G

def jacobian_B(mu, u, dt):
    """
    Jacobian B_t = df/du of the motion model w.r.t control input.
    """
    _, _, theta = mu.flatten()

    B = np.array([
        [dt * np.cos(theta), 0],
        [dt * np.sin(theta), 0],
        [0, dt]
    ])
    return B

def observation_model(mu):
    """
    Observation model h(mu) -> returns expected observation.
    Now includes x, y, theta.
    """
    x, y, theta = mu.flatten()
    return np.array([[x], [y], [theta]])

def jacobian_H(mu):
    """
    Jacobian H_t = dh/dmu for observation model.
    Includes x, y, theta.
    """
    return np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])
