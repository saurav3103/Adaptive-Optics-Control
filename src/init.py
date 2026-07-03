import numpy as np
import matplotlib.pyplot as plt

# ── Physical parameters ──────────────────────────────────────
lam = 633e-9          # wavelength [m] — HeNe laser
f   = 0.1             # focal length [m]

# ── Slit nominal state ───────────────────────────────────────
a0  = 0.5e-3          # nominal slit width [m]
c0  = 0.0             # nominal slit center [m]

# ── Screen ───────────────────────────────────────────────────
x   = np.linspace(-5e-3, 5e-3, 1000)   # screen coordinates [m]

# ── ROI on screen ────────────────────────────────────────────
roi_left  = -0.5e-3   # [m]
roi_right =  0.5e-3   # [m]
roi_mask  = (x >= roi_left) & (x <= roi_right)

# ── Simulation time ──────────────────────────────────────────
dt   = 1e-3           # timestep [s]
T    = 30.0            # total time [s]
time = np.arange(0, T, dt)

print(f"Fresnel number at nominal width: {a0**2 / (lam * f):.1f}")
print(f"Screen range: {x[0]*1e3:.1f} mm to {x[-1]*1e3:.1f} mm")
print(f"ROI: {roi_left*1e3:.1f} mm to {roi_right*1e3:.1f} mm")
print(f"Timesteps: {len(time)}")
