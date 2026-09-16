import numpy as np

# ============================================================
# Parameters from Zhao et al. paper
# ============================================================

MASS = 4.5          # canopy 0.5 + payload 4 kg
RHO = 1.225         # air density [kg/m^3]
G = 9.81

b = 3.0             # canopy span [m]
c = 1.0             # canopy chord [m]
d = 0.1             # control-line length [m]

S = b * c           # approximate forced area [m^2]

# Aerodynamic coefficients from Table 3
CL0 = 0.5
CL_ALPHA = 1.7190
CL_DELTA = 0.0001

CD0 = 0.2
CD_ALPHA2 = 0.7
CD_DELTA = 0.0001

CM0 = 0.1397
CM_ALPHA = -1.4308
CM_Q = -0.2251

# Identified roll/yaw coefficients from Table 1
CL_PHI = -0.04
CL_P = -0.08
CL_DELTA_A = -0.00001

CN_R = -0.012
CN_DELTA_A = -0.00008

# ------------------------------------------------------------
# The paper gives the inertia MATRIX form but not numerical
# IXX/IYY/IZZ/IXZ values.
#
# These are therefore explicit modelling assumptions.
# ------------------------------------------------------------

IXX = 2.0
IYY = 2.0
IZZ = 2.0
IXZ = 0.0

I = np.array([
    [IXX, 0.0, IXZ],
    [0.0, IYY, 0.0],
    [IXZ, 0.0, IZZ]
])

I_INV = np.linalg.inv(I)


def rotation_inertial_to_body(phi, theta, psi):
    """
    B_d-s from Eq. (1) of the paper.
    """

    cp = np.cos(phi)
    sp = np.sin(phi)

    ct = np.cos(theta)
    st = np.sin(theta)

    cy = np.cos(psi)
    sy = np.sin(psi)

    return np.array([
        [
            ct * cy,
            ct * sy,
            -st
        ],
        [
            sp * st * cy - cp * sy,
            sp * st * sy + cp * cy,
            sp * ct
        ],
        [
            cp * st * cy + sp * sy,
            cp * st * sy - sp * cy,
            cp * ct
        ]
    ])


def dynamics(state, delta_a):
    """
    Paper's nonlinear 6-DOF model.

    State:
        x, y, z,
        phi, theta, psi,
        u, v, w,
        p, q, r
    """

    x, y, z = state[0:3]
    phi, theta, psi = state[3:6]
    u, v, w = state[6:9]
    p, q, r = state[9:12]

    V = np.array([u, v, w])
    W = np.array([p, q, r])

    speed = max(np.linalg.norm(V), 1e-6)

    # Angle of attack
    alpha = np.arctan2(w, u)

    # --------------------------------------------------------
    # Gravity force
    # Paper's inertial z-axis points DOWN.
    # --------------------------------------------------------

    B = rotation_inertial_to_body(phi, theta, psi)

    F_gravity = B @ np.array([0.0, 0.0, MASS * G])

    # --------------------------------------------------------
    # Aerodynamic forces — Eq. (7)
    # --------------------------------------------------------

    CL = CL0 + CL_ALPHA * alpha + CL_DELTA * delta_a

    CD = CD0 + CD_ALPHA2 * alpha**2 + CD_DELTA * delta_a

    q_dyn = 0.5 * RHO * S * speed

    F_lift = (
        q_dyn * CL *
        np.array([w, 0.0, -u])
    )

    F_drag = (
        -q_dyn * CD *
        np.array([u, v, w])
    )

    F_aero = F_lift + F_drag

    # --------------------------------------------------------
    # Aerodynamic moments — Eq. (8)
    # --------------------------------------------------------

    roll_moment = (
        CL_PHI * b * phi
        + CL_P * b**2 * p / (2 * speed)
        + CL_DELTA_A * delta_a * b / d
    )

    pitch_moment = (
        CM0 * c
        + CM_ALPHA * c * alpha
        + CM_Q * c**2 * q / (2 * speed)
    )

    yaw_moment = (
        CN_R * b**2 * r / (2 * speed)
        + CN_DELTA_A * delta_a * b / d
    )

    M_aero = (
        0.5 * RHO * S * speed**2 *
        np.array([
            roll_moment,
            pitch_moment,
            yaw_moment
        ])
    )

    # --------------------------------------------------------
    # Translational dynamics — Eq. (11)
    #
    # m(Vdot + W x V) = F
    # --------------------------------------------------------

    V_dot = (
        (F_gravity + F_aero) / MASS
        - np.cross(W, V)
    )

    # --------------------------------------------------------
    # Rotational dynamics — Eq. (12)
    #
    # I Wdot + W x (I W) = M
    # --------------------------------------------------------

    W_dot = I_INV @ (
        M_aero - np.cross(W, I @ W)
    )

    # --------------------------------------------------------
    # Position derivatives — Eq. (2)
    # --------------------------------------------------------

    position_dot = B.T @ V

    # --------------------------------------------------------
    # Euler angle derivatives — Eq. (3)
    # --------------------------------------------------------

    # Protect against theta = +/- 90 degrees
    cos_theta = max(abs(np.cos(theta)), 1e-5) * np.sign(
        np.cos(theta)
    )

    tan_theta = np.sin(theta) / cos_theta

    angle_dot = np.array([
        p + np.sin(phi) * tan_theta * q
          + np.cos(phi) * tan_theta * r,

        np.cos(phi) * q - np.sin(phi) * r,

        np.sin(phi) / cos_theta * q
          + np.cos(phi) / cos_theta * r
    ])

    return np.concatenate([
        position_dot,
        angle_dot,
        V_dot,
        W_dot
    ])
