"""
Robot Dynamics and Control Assignment 1a: PID controller and step response
-------------------------------------------------------------------------------
DESCRIPTION:
1-DoF mass-damper plant controlled by a PID controller.

Creates and tunes PID controllers for three different cases to achieve step
responses of a mass-damper plant that include:
1) overshoot
2) no overshoot
3) instability
-------------------------------------------------------------------------------
"""

import numpy as np
import matplotlib.pyplot as plt

# Constants
m, b, g = 1.0, 1.0, 9.81  # mass, damping, gravity
dt, T, xr = 0.001, 10.0, 1.0  # time step, simulation time, reference position

# Initialize states for each case
state1, state2, state3 = [], [], []

# Initialize states for each case
state1, state2, state3 = [], [], []


def simulate(Kp, Ki, Kd):
    se, pe, t, x, dx, ddx = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    states = []

    for i in range(int(T / dt)):
        error = xr - x
        se += error * dt
        de = (error - pe) / dt
        F = Kp * error + Ki * se + Kd * de
        pe = error

        ddx = F / m - dx * b / m
        dx += ddx * dt
        x += dx * dt
        t += dt
        states.append([t, x, dx, ddx])

    return np.array(states)


# Simulate cases
state1 = simulate(20, 15, 1)  # overshoot
state2 = simulate(0.5, 0.5, 50)  # no overshoot
state3 = simulate(5, 13, 1)  # instability

# Plotting
plt.figure(figsize=(10, 5))

plt.subplot()
plt.title("Overshoot")
plt.plot(state1[:, 0], state1[:, 1])
plt.ylabel("position [m]")

plt.subplot()
plt.title("No Overshoot")
plt.plot(state2[:, 0], state2[:, 1])
plt.ylabel("position [m]")

plt.subplot()
plt.title("Instability")
plt.plot(state3[:, 0], state3[:, 1])
plt.ylabel("position [m]")
plt.xlabel("time [s]")

plt.legend(
    [
        "Overshoot: Kp = 20, Ki = 15, Kd = 1",
        "No Overshoot: Kp = 0.5, Ki = 0.5, Kd = 50",
        "Instability: Kp = 5, Ki = 13, Kd = 1",
    ]
)

plt.show()
