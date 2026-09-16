import numpy as np


def guidance(state, target):

    x, y, z = state[0:3]
    psi = state[5]

    tx, ty, tz = target

    dx = tx - x
    dy = ty - y

    desired_heading = np.arctan2(dy, dx)

    heading_error = (
        desired_heading - psi + np.pi
    ) % (2 * np.pi) - np.pi

    K = 0.5

    delta = K * heading_error

    return np.clip(delta, -1.0, 1.0)
