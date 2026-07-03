# Adaptive Optics-Inspired Control Simulation

A Python simulation of a closed-loop optical control system, built as preparation for the MSc Systems and Control program at TU Delft (DCSC). The project models wavefront correction in adaptive optics (AO) using a slit-lens-focal plane setup, with progressively sophisticated controllers.

## Motivation

Adaptive optics systems — used in astronomical telescopes and semiconductor lithography (ASML EUV) — correct optical aberrations in real time using a sense-compute-actuate loop. This simulation captures the core control problem: a mechanical disturbance corrupts the diffraction pattern on a screen, and the controller must restore illumination to a target region of interest (ROI).

The setup maps directly to real AO:

| Simulation | AO Equivalent |
|---|---|
| Slit center disturbance | Atmospheric tip/tilt |
| Slit width disturbance | Higher-order wavefront aberration |
| PI / D+ / LQG controller | AO control loop |
| ROI mean intensity | Strehl ratio / PSF quality |

## Optical Setup

```
Plane wave → Disturbed Slit → Thin Lens (f = 0.1m) → Focal Plane (screen)
```

A monochromatic plane wave (λ = 633 nm, HeNe laser) passes through a slit disturbed by a mechanical jerk. A thin lens performs an exact Fourier transform at the focal plane. The intensity pattern is:

$$I(x, t) = \frac{a(t)^2}{a_0^2} \cdot \text{sinc}^2\left(\frac{a(t)(x - c(t))}{\lambda f}\right)$$

The disturbance model is:

$$a(t) = a_0 + A_a \sin(2\pi f_a t) + \eta(t), \quad f_a = 2.0 \text{ Hz}$$
$$c(t) = c_0 + A_c \sin(2\pi f_c t + \phi) + \eta(t), \quad f_c = 1.5 \text{ Hz}$$

## Controllers

### 1. PI Controller
A proportional-integral controller on slit position. Derivative term excluded — amplifies noise at small timesteps (dt = 1ms).

### 2. Static System Identification + D+ Controller
Interaction matrix D identified via actuator poke testing (finite differences). Control matrix D⁺ = pinv(D) maps sensor error to actuator correction. Mathematically equivalent to least-squares regression.

**D matrix (2 actuators × 2 ROI zones):**
- Column 1 (center poke): antisymmetric response `[-1881, +1881]`
- Column 2 (width poke): symmetric response `[+252, +252]`
- Condition number: 7.48 — well-conditioned

### 3. Actuator Lag (First-Order Dynamics)
Real actuators have bandwidth limits modeled as a first-order lag:

$$\tau \dot{c}_{act} + c_{act} = c_{cmd}, \quad \tau = 0.1\text{s} \implies f_c = 1.6\text{ Hz}$$

With disturbances at 1.5–2.0 Hz near the actuator cutoff, D+ performance degrades significantly.

### 4. LQG (Kalman Filter + Feedforward)
Disturbance frequencies identified from open-loop PSD (Welch's method). A harmonic oscillator state-space model is fit and used in a steady-state Kalman filter to estimate and predict the disturbance. The control law applies feedforward cancellation:

$$u_c = -\hat{x}_1, \quad u_a = -(\hat{x}_3 - a_0)$$

## Results

| Controller | ROI Mean Intensity | Std | vs Open Loop |
|---|---|---|---|
| Open loop | 0.1224 | 0.0186 | baseline |
| PI control | 0.1230 | 0.0037 | 5x std reduction |
| D+ (no lag) | 0.1237 | 0.0004 | 46x std reduction |
| D+ (with lag) | 0.1235 | 0.0147 | performance degraded |
| LQG | 0.1501 | 0.0150 | mean recovered despite lag |

**Key insight:** D+ significantly outperforms PI near the identification point, but degrades outside the linear regime and under actuator lag. LQG with a disturbance model recovers mean intensity by predicting the sinusoidal disturbance, though std remains limited by actuator bandwidth.

## Project Structure

```
ao-control-simulation/
├── README.md
├── requirements.txt
├── notebooks/
│   └── ao_simulation.ipynb
├── src/
│   ├── actuator_dynamics.py
│   ├── closed_loop_control_with_PI_controller.py
│   ├── disturbance_model.py
│   ├── dynamic_lqg_controller.py
│   ├── open_loop_control.py
│   ├── optics.py
│   ├── psd_analysis.py
│   ├── system_identification.py
├── results/
│   ├── actuator_lag.png
│   ├── lqg_control.png
│   ├── open_loop.png
│   ├── open_loop_vs_closed_loop_PI.png
│   ├── open_loop_vs_closed_loop_PI_vs_D+_control.png
│   └── psd_analysis.png
└── docs/
    └── writeup.md
```

## Key Concepts Demonstrated

- Free-space propagation as an LTI system; lens as exact Fourier transform operator
- Interaction matrix calibration via poke-and-measure (standard AO procedure)
- Pseudoinverse control = least-squares solution (connection to regression)
- First-order lag as low-pass filter; actuator bandwidth vs disturbance frequency
- Welch PSD for disturbance frequency identification from open-loop data
- Kalman filter for state estimation with harmonic oscillator disturbance model

## Connection to Research

This simulation is directly motivated by wavefront control methods used in:
- Verhaegen group (N4CI, TU Delft DCSC) — data-driven AO control
- ASML EUV lithography — sub-nm wavefront correction at high bandwidth
- Relevant paper: Hinnen et al., "A data-driven H2-optimal control approach for adaptive optics"

## Requirements

See `requirements.txt`. Python 3.9+.

## Author

Saurav — Incoming MSc Systems and Control, TU Delft (2026)
