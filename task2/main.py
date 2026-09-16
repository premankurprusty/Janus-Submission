import numpy as np
import matplotlib.pyplot as plt

from scipy.integrate import solve_ivp

from dynamics import dynamics
from guidance import guidance


# ============================================================
# Simulation settings
# ============================================================

TARGET = np.array([0.0, 0.0, 0.0])

INITIAL_ALTITUDE = 700.0

# Random starting x/y as required
rng = np.random.default_rng()

x0 = rng.uniform(-500, 500)
y0 = rng.uniform(-500, 500)

# Paper uses inertial z positive DOWN.
# Therefore 700 m altitude corresponds to z = -700.
z0 = -INITIAL_ALTITUDE


# Initial attitude
phi0 = 0.0
theta0 = 0.0
psi0 = np.random.uniform(-np.pi, np.pi)


# Paper's initial velocity example:
# u = 6 m/s, v = 0, w = 3 m/s
u0 = 6.0
v0 = 0.0
w0 = 3.0

p0 = 0.0
q0 = 0.0
r0 = 0.0


state = np.array([
    x0, y0, z0,
    phi0, theta0, psi0,
    u0, v0, w0,
    p0, q0, r0
])


# ============================================================
# Simulation
# ============================================================

dt = 0.1
MAX_TIME = 300


history = [state.copy()]
times = [0.0]

t = 0.0


while t < MAX_TIME:

    # Stop once close enough to target
    position = state[0:3]

    if np.linalg.norm(position - TARGET) < 10:
        print("Target reached!")
        break

    delta = guidance(state, TARGET)

    # RK4 integration
    k1 = dynamics(state, delta)
    k2 = dynamics(state + 0.5 * dt * k1, delta)
    k3 = dynamics(state + 0.5 * dt * k2, delta)
    k4 = dynamics(state + dt * k3, delta)

    state = state + (
        dt / 6 *
        (k1 + 2*k2 + 2*k3 + k4)
    )

    t += dt

    history.append(state.copy())
    times.append(t)

    # Safety: stop if we hit the ground
    if state[2] >= 0:
        print("Ground reached.")
        break


history = np.array(history)


# ============================================================
# Convert paper's DOWN-positive z into altitude for plotting
# ============================================================

X = history[:, 0]
Y = history[:, 1]

# paper z is positive downward
ALTITUDE = -history[:, 2]


# ============================================================
# Plot
# ============================================================

fig = plt.figure(figsize=(10, 8))

ax = fig.add_subplot(111, projection="3d")

ax.plot(
    X,
    Y,
    ALTITUDE,
    linewidth=2,
    label="Paraglider path"
)

ax.scatter(
    [0],
    [0],
    [0],
    s=100,
    marker="*",
    label="Target"
)

ax.scatter(
    [X[0]],
    [Y[0]],
    [ALTITUDE[0]],
    s=60,
    label="Start"
)

ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Altitude [m]")

ax.set_title("Paraglider Path Planning")

ax.legend()

plt.show()
