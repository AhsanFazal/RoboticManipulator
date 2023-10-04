"""
Robot Dynamics and Control Assignment 1b: frequency response of PID controller
-------------------------------------------------------------------------------
DESCRIPTION:
1-DoF mass-damper plant controlled by a PID controller.

Analyses the frequency response of the provided PID controller for a mass-damper
plant using the Bode plot. Experimentally sampling the actual position (output)
for a given sine reference position signal (input) at frequencies in Hertz (Hz)
provided in vector freq. Displays magnitude in decibels (dB) and phase in
degrees.
-------------------------------------------------------------------------------

"""


import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks  # useful for analysis

# Constants
m, b, g = 1.0, 1.0, 9.81
dt, T = 0.001, 10.0
freq = np.linspace(0.1, 10, 100)
Kp, Ki, Kd = 10.0, 5.0, 1.0


# Initialize
bode_gain, bode_phase = [], []
t = np.linspace(0, T, int(T / dt))


def simulate_frequency_response(frequency, time):
    x, dx, ddx = 0.0, 0.0, 0.0
    se, pe = 0.0, 0.0
    output_signals = []
    input_signals = []

    for ti in time:
        input_signal = np.sin(
            2 * np.pi * frequency * ti
        )  # Generate input for this time step
        input_signals.append(input_signal)
        error = input_signal - x
        se += error * dt
        de = (error - pe) / dt
        u = Kp * error + Ki * se + Kd * de
        ddx = (u - b * dx) / m
        dx += ddx * dt
        x += dx * dt
        output_signals.append(x)
        pe = error

    input_peaks, _ = find_peaks(input_signals)
    output_peaks, _ = find_peaks(output_signals)
    gain = 20 * np.log10(
        np.abs(output_signals[output_peaks[0]]) / np.abs(input_signals[input_peaks[0]])
    )
    phase = (output_peaks[0] - input_peaks[0]) * 360 / len(time)
    return gain, phase


# Main logic for frequency response
for f in freq:
    gain, phase = simulate_frequency_response(f, t)
    bode_gain.append(gain)
    bode_phase.append(phase)

# Plotting
plt.figure(figsize=(10, 5))

plt.subplot(211)
plt.title("Bode Plot: Gain")
plt.plot(freq, bode_gain, "xb")
plt.xscale("log")
plt.xlabel("Frequency [Hz]")
plt.ylabel("Gain [dB]")
plt.grid(True)

plt.subplot(212)
plt.title("Bode Plot: Phase")
plt.plot(freq, bode_phase, "xr")
plt.xscale("log")
plt.xlabel("Frequency [Hz]")
plt.ylabel("Phase [degrees]")
plt.grid(True)

plt.tight_layout()
plt.show()
