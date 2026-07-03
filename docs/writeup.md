# Technical Writeup: Adaptive Optics-Inspired Control Simulation

**Author:** Saurav  
**Program:** Incoming MSc Systems and Control, TU Delft (DCSC), August 2026  
**Context:** Pre-MSc portfolio project — bridging optics fundamentals and control engineering

---

## 1. Motivation

Adaptive optics (AO) is the technology that corrects optical aberrations in real time. Originally developed for astronomical telescopes, it now appears in semiconductor lithography (ASML EUV systems), ophthalmology, and free-space optical communications. The core problem is always the same: a disturbance corrupts the wavefront, a sensor measures the error, and a controller drives actuators to correct it — all within milliseconds.

This project builds a simulation of that loop from first principles. Rather than simulating a full AO system (which requires Zernike decomposition, Shack-Hartmann sensors, and deformable mirror influence functions), the simulation uses a single slit as the aperture and its Fraunhofer diffraction pattern as the output field. This reduces the problem to two scalar actuators and one measurable output field — small enough to build in a Jupyter notebook, rich enough to demonstrate every major control concept relevant to real AO.

The motivation for this specific setup is direct: the N4CI group at TU Delft (Prof. Verhaegen) works on data-driven control for AO systems, and ASML uses wavefront sensing and control in their lithography machines. Understanding the control loop at this level is a prerequisite for engaging with that research.

---

## 2. Optical Setup and Physical Model

### 2.1 The Optical Chain

A monochromatic plane wave (λ = 633 nm, HeNe laser) is incident on a slit. The slit has two degrees of freedom: its width $a(t)$ and its center position $c(t)$. A thin lens of focal length $f = 0.1$ m is placed immediately after the slit. The screen (detector plane) is at the focal plane of the lens.

```
Plane wave → Slit [a(t), c(t)] → Thin Lens [f = 0.1m] → Focal Plane (screen)
```

### 2.2 Why a Lens?

Without a lens, propagation from slit to screen is governed by the Fresnel diffraction integral — an approximation valid for intermediate distances. A thin lens placed after the slit performs an *exact* Fourier transform of the aperture function at its focal plane, regardless of propagation distance. This is a standard result in Fourier optics (Goodman, Ch. 5).

The Fresnel number at nominal slit width is:

$$N_F = \frac{a_0^2}{\lambda f} = \frac{(0.5 \times 10^{-3})^2}{633 \times 10^{-9} \times 0.1} \approx 3.9$$

This is not deep in the Fraunhofer regime ($N_F \ll 1$), confirming that a lens is necessary to achieve the exact Fourier relationship — the far-field approximation alone would not suffice at this distance.

### 2.3 Intensity at the Focal Plane

For a slit of width $a$ centered at $c$, the field at the focal plane is proportional to the Fourier transform of a rect function. The normalized intensity is:

$$I(x, t) = \frac{a(t)^2}{a_0^2} \cdot \text{sinc}^2\left(\frac{a(t)(x - c(t))}{\lambda f}\right)$$

Key observations:
- Shifting $c(t)$ translates the entire pattern laterally on the screen
- Increasing $a(t)$ narrows the central lobe (more spatial confinement → narrower Fourier peak)
- The relationship between $(a, c)$ and $I(x)$ is **nonlinear** — the sinc function is smooth but not linear

### 2.4 Connection to Free-Space as LTI System

From the TU Delft Advanced Optical Imaging course notes: free-space propagation can be written as a linear, shift-invariant (LTI) operation in the spatial frequency domain via the angular spectrum transfer function:

$$H(f_x, f_y) = \exp\left[i2\pi z \sqrt{\frac{1}{\lambda^2} - f_x^2 - f_y^2}\right]$$

The lens performs the Fourier transform that makes this explicit at the focal plane. This LTI interpretation is the bridge from optics to control — the optical system is a plant with a known transfer function.

### 2.5 Disturbance Model

The mechanical jerk disturbs both slit parameters:

$$a(t) = a_0 + A_a \sin(2\pi f_a t) + \eta_a(t), \quad A_a = 0.1 \text{ mm}, \quad f_a = 2.0 \text{ Hz}$$
$$c(t) = c_0 + A_c \sin(2\pi f_c t + \phi) + \eta_c(t), \quad A_c = 0.3 \text{ mm}, \quad f_c = 1.5 \text{ Hz}$$

where $\eta(t) \sim \mathcal{N}(0, \sigma^2)$ with $\sigma = 0.02$ mm is additive measurement noise and $\phi = \pi/4$ introduces a phase offset between the two disturbances.

The position disturbance (0.3 mm amplitude) dominates over the width disturbance (0.1 mm amplitude). This reflects real AO systems where tip/tilt (position) correction is the dominant contribution to wavefront error.

### 2.6 Control Objective

Define a region of interest (ROI) on the screen: $x \in [-0.5, +0.5]$ mm. The control objective is:

$$\text{maximize } \bar{I}_{ROI}(t) = \frac{1}{|\text{ROI}|}\int_{\text{ROI}} I(x, t)\, dx$$

In practice: keep ROI mean intensity high and variance low despite the disturbance.

---

## 3. Open Loop Baseline

With no controller, the disturbance drives the slit center ±0.3 mm and width ±0.1 mm around nominal. The ROI intensity varies between 0.08 and 0.16 — nearly 2× variation.

**Open loop result:** mean = 0.1224, std = 0.0186

The ROI intensity is dominated by the center disturbance: when $c(t)$ shifts the sinc pattern away from the ROI, intensity drops. This confirms that tip/tilt correction (center control) is the higher priority actuator.

---

## 4. PI Controller

### 4.1 Design

A proportional-integral controller on slit position:

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau$$

with separate controllers for center ($K_p = 0.8$, $K_i = 0.1$) and width ($K_p = 0.8$, $K_i = 0.1$).

The derivative term was excluded after empirical observation: with $dt = 1$ ms, the discrete derivative $(e[k] - e[k-1])/dt$ amplifies noise by $10^3$. This caused instability — position std *doubled* when Kd was included. This is a known issue in digital control implementations and is the reason real AO systems either filter the error signal or use pure PI.

### 4.2 Tuning Process

Gains were found by starting with P-only control ($K_p = 0.5$, $K_i = K_d = 0$) and verifying that position std halved — exactly as predicted by theory for $u = K_p e$, $c_{act} = c_{dist} + K_p(-c_{dist}) = (1-K_p)c_{dist}$.

Anti-windup was implemented to prevent integral saturation during large transients.

### 4.3 Result

**PI result:** mean = 0.1230, std = 0.0037 — **5× std reduction over open loop**

The integral term eliminates steady-state offset and keeps the actuated center near zero. Residual variance comes from the noise term $\eta(t)$ which the PI cannot fully reject.

---

## 5. Static System Identification and D⁺ Controller

### 5.1 Motivation

The PI controller was manually tuned and uses no model of how actuator motion maps to intensity. A model-based approach — if the model is accurate — should outperform heuristic tuning.

### 5.2 Interaction Matrix Identification

Define a measurement vector $\mathbf{s} = [s_L, s_R]^T$ where $s_L$, $s_R$ are mean intensities in the left and right halves of the ROI respectively. This splits the scalar ROI into two zones, giving a square 2×2 system.

The interaction matrix $D$ relates actuator commands to sensor measurements:

$$\Delta \mathbf{s} = D \cdot \Delta \mathbf{u}, \quad \mathbf{u} = [\Delta c, \Delta a]^T$$

$D$ is identified by poking each actuator individually with amplitude $\delta$ and recording the response:

$$D_{:,i} = \frac{\mathbf{s}(\mathbf{u}_0 + \delta \mathbf{e}_i) - \mathbf{s}(\mathbf{u}_0 - \delta \mathbf{e}_i)}{2\delta}$$

This is a central finite difference — symmetric poking cancels the first-order nonlinearity, improving accuracy.

**Identified D matrix:**

$$D = \begin{bmatrix} -1881 & 252 \\ +1881 & 252 \end{bmatrix}$$

The structure confirms the physics:
- Column 1 (center poke): antisymmetric — shifting center right decreases left ROI, increases right ROI
- Column 2 (width poke): symmetric — changing width affects both zones equally
- Condition number = 7.48 — well-conditioned, inversion is reliable

**Connection to regression:** The interaction matrix identification problem is mathematically identical to linear regression. $D$ is the design matrix, the poke amplitudes are inputs, the sensor responses are outputs, and $D^+$ is the least-squares solution. This is not coincidental — system identification *is* regression applied to dynamical systems.

### 5.3 D⁺ Controller

The control law is:

$$\mathbf{u}_{cmd} = -D^+ \cdot (\mathbf{s}_{measured} - \mathbf{s}_{nominal})$$

where $D^+ = (D^T D)^{-1} D^T$ is the Moore-Penrose pseudoinverse. This finds the minimum-norm actuator command that best corrects the measured sensor deviation.

### 5.4 Validity

The linear model is valid near the identification operating point. Validation with a small perturbation ($\Delta c = 0.01$ mm) gives ~10% relative error. At larger perturbations ($\Delta c = 0.1$ mm) the error rises to ~50% — the sinc nonlinearity becomes significant. In real AO systems the interaction matrix is re-calibrated periodically as the operating point drifts.

### 5.5 Result

**D⁺ result (no lag):** mean = 0.1237, std = 0.0004 — **46× std reduction over open loop**

The model-based controller significantly outperforms PI because it uses the identified relationship between each actuator and each measurement zone, applying precisely the right correction without manual gain tuning.

---

## 6. Actuator Dynamics

### 6.1 Physical Model

Real actuators do not respond instantaneously. A piezoelectric actuator driving a mechanical slit element is well-modeled as a mass-spring-damper:

$$m\ddot{x} + b\dot{x} + kx = F_{cmd}$$

For a heavily damped actuator (typical of precision mechanics), the inertia term $m\ddot{x}$ is negligible compared to damping $b\dot{x}$. The equation reduces to a first-order lag:

$$\tau \dot{x} + x = x_{cmd}, \quad \tau = \frac{b}{k}$$

Taking the Fourier transform gives the transfer function:

$$\frac{X(\omega)}{X_{cmd}(\omega)} = \frac{1}{1 + j\omega\tau}$$

This is a low-pass filter with cutoff frequency $f_c = 1/(2\pi\tau)$. Disturbances below $f_c$ are tracked well; disturbances above $f_c$ are attenuated and phase-shifted.

### 6.2 Discrete-Time Implementation

$$c_{act}[k] = \alpha \cdot c_{act}[k-1] + (1-\alpha) \cdot c_{cmd}[k], \quad \alpha = e^{-dt/\tau}$$

With $\tau = 0.1$ s, the cutoff frequency is $f_c = 1.6$ Hz. The disturbances at 1.5 Hz and 2.0 Hz straddle this cutoff — the actuator partially tracks the 1.5 Hz component and significantly lags the 2.0 Hz component.

### 6.3 Why This Degrades D⁺

The D⁺ controller computes the correct command instantaneously, but the actuator cannot execute it fast enough. By the time the actuator has applied 95% of the commanded correction ($t = 3\tau = 0.3$ s), the sinusoidal disturbance (period = 0.5 s at 2 Hz) has already reversed direction. The actuator is always chasing a target that has moved.

### 6.4 Result

**D⁺ with lag:** mean = 0.1235, std = 0.0147 — **37× std degradation vs ideal D⁺**

The mean is barely affected (actuator reaches steady state eventually) but variance is dominated by the lag-induced tracking error.

---

## 7. Disturbance Identification via PSD

### 7.1 Motivation

The Kalman filter (next section) requires a model of the disturbance dynamics. To build this model without assuming prior knowledge, the disturbance spectrum is identified from open-loop sensor data.

### 7.2 Welch's Method

The Power Spectral Density (PSD) estimates how signal power is distributed across frequency:

$$S(f) = \frac{1}{T}|\mathcal{F}\{c(t)\}|^2$$

Welch's method reduces variance by splitting the signal into overlapping windows, computing the FFT of each, and averaging the squared magnitudes. The frequency resolution is $\Delta f = f_s / N_{seg}$ where $N_{seg}$ is the window length.

**Key tradeoff:** longer windows → finer frequency resolution → fewer windows to average → noisier estimate. With 30 s of data at 1 kHz sampling, $N_{seg} = 8192$ gives $\Delta f = 0.12$ Hz — sufficient to resolve 1.5 Hz and 2.0 Hz as separate peaks.

### 7.3 Result

Identified peaks:
- $c(t)$: dominant peak at **1.465 Hz ≈ 1.5 Hz**, SNR ≈ $10^5$
- $a(t)$: dominant peak at **1.953 Hz ≈ 2.0 Hz**, SNR ≈ $10^4$

Both peaks are sharp (not broad), confirming the disturbance is sinusoidal rather than colored noise. The noise floor is 4–5 orders of magnitude below the signal peaks.

The small discrepancy from true values (1.465 vs 1.5 Hz) is frequency bin quantization — the nearest bin to 1.5 Hz at 0.12 Hz resolution is 1.464 Hz.

---

## 8. LQG Controller (Kalman Filter + Feedforward)

### 8.1 Disturbance State-Space Model

A pure sinusoid at frequency $\omega$ satisfies $\ddot{x} = -\omega^2 x$. Defining state variables $x_1 = c_{dist}$, $x_2 = \dot{c}_{dist}$:

$$\begin{bmatrix}\dot{x}_1 \\ \dot{x}_2\end{bmatrix} = \begin{bmatrix}0 & 1 \\ -\omega_c^2 & 0\end{bmatrix}\begin{bmatrix}x_1 \\ x_2\end{bmatrix}$$

Stacking both disturbances into a 4-dimensional state:

$$\mathbf{x} = [c_{dist},\ \dot{c}_{dist},\ a_{dist},\ \dot{a}_{dist}]^T$$

$$A = \begin{bmatrix}0 & 1 & 0 & 0 \\ -\omega_c^2 & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & -\omega_a^2 & 0\end{bmatrix}, \quad C = \begin{bmatrix}1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0\end{bmatrix}$$

The off-diagonal blocks are zero — the two disturbances are physically independent.

$C$ reflects what the sensor physically measures: slit position ($c$, $a$) but not velocity ($\dot{c}$, $\dot{a}$). Velocity is an internal state estimated by the Kalman filter.

The continuous-time model is discretized via matrix exponential: $A_d = e^{A \cdot dt}$.

### 8.2 Steady-State Kalman Filter

The Kalman filter estimates the full state $\hat{\mathbf{x}}$ from noisy position measurements. The steady-state gain is found by iterating the discrete Riccati equation to convergence:

**Predict:**
$$\hat{\mathbf{x}}^- = A_d \hat{\mathbf{x}}, \quad P^- = A_d P A_d^T + Q$$

**Update:**
$$K = P^- C^T (C P^- C^T + R)^{-1}$$
$$\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + K(\mathbf{y} - C\hat{\mathbf{x}}^-)$$
$$P = (I - KC)P^-$$

Noise covariances: $Q = \text{diag}(10^{-6}, 10^{-6}, 10^{-8}, 10^{-8})$ (process noise), $R = \text{diag}(10^{-8}, 10^{-8})$ (measurement noise). The ratio $Q/R$ determines how much the filter trusts the model vs the measurements.

### 8.3 Feedforward Control Law

Since the disturbance model has no control input channel (we cannot control the atmosphere — only estimate and cancel it), the control law is direct feedforward cancellation of the estimated disturbance:

$$u_c = -\hat{x}_1 = -\hat{c}_{dist}$$
$$u_a = -(\hat{x}_3 - a_0) = -(\hat{a}_{dist} - a_0)$$

The Kalman filter predicts where the disturbance will be one step ahead — the actuator is commanded early to compensate for its own lag. This predictive property is the key advantage of LQG over purely reactive controllers like PI or D⁺.

### 8.4 Result

**LQG result:** mean = 0.1501, std = 0.0150

The mean intensity increased significantly (0.122 → 0.150) compared to open loop and D⁺-with-lag, confirming the Kalman filter is correctly estimating and partially predicting the disturbance. The std did not improve substantially over D⁺-with-lag — the actuator bandwidth ($f_c = 1.6$ Hz) remains the binding constraint on dynamic performance.

---

## 9. Summary and Discussion

### 9.1 Results Table

| Controller | Mean ROI Intensity | Std | Key Limitation |
|---|---|---|---|
| Open loop | 0.1224 | 0.0186 | No correction |
| PI | 0.1230 | 0.0037 | Manual tuning, no plant model |
| D⁺ (no lag) | 0.1237 | 0.0004 | Valid only near operating point |
| D⁺ (with lag) | 0.1235 | 0.0147 | Actuator bandwidth |
| LQG | 0.1501 | 0.0150 | Actuator bandwidth (std); lag compensation incomplete |

### 9.2 Key Insights

**PI vs D⁺:** D⁺ outperforms PI near the identification point by using the learned plant model. PI is robust to operating point changes but imprecise. D⁺ is precise but brittle — valid only within ~10% perturbation of the identification point. In practice both are used together: D⁺ for nominal operation, with re-identification when conditions drift.

**Actuator bandwidth is the binding constraint:** The first-order lag with $\tau = 0.1$ s sets a cutoff at 1.6 Hz. With disturbances at 1.5–2.0 Hz straddling the cutoff, no purely reactive controller can fully reject the disturbance — the actuator physically cannot keep up. This motivates predictive control.

**LQG recovers mean, not variance:** The Kalman filter improves steady-state estimation (mean) but the actuator lag limits transient rejection (std). To further reduce std requires either: (a) a faster actuator ($\tau < 0.01$ s for $f_c > 15$ Hz), (b) explicit lag compensation in the control law (Smith predictor), or (c) including actuator dynamics in the Kalman state.

### 9.3 Limitations

- The LQG implementation uses Kalman feedforward rather than full LQR optimal control. A complete LQG formulation would include the actuator dynamics in the state model and solve the LQR problem jointly.
- Static SI is only valid locally. No online re-identification was implemented.
- The simulation uses a 1D slit rather than a 2D aperture. Real AO systems require Zernike decomposition over a circular aperture and a full deformable mirror influence matrix.
- Measurement noise is white Gaussian. Real wavefront sensors have correlated noise and quantization effects.

### 9.4 Connection to Real AO Systems

The control loop built here maps directly onto real AO:

| This Simulation | Real AO System |
|---|---|
| Slit center $c(t)$ | Atmospheric tip/tilt |
| Slit width $a(t)$ | Higher-order Zernike modes |
| Interaction matrix $D$ | DM influence matrix |
| D⁺ control law | Standard AO reconstructor |
| Kalman filter | Optimal wavefront predictor (Hinnen et al.) |
| ROI mean intensity | Strehl ratio |

The data-driven approach to identifying $D$ and building a Kalman predictor from open-loop PSD data is directly related to the methodology in:

> Hinnen, K., Verhaegen, M., & Doelman, N. (2008). *A data-driven H2-optimal control approach for adaptive optics.* IEEE Transactions on Control Systems Technology.

That paper formalizes the identification-to-control pipeline demonstrated here, using subspace methods (N4SID) for dynamic model identification rather than the static interaction matrix approach used in this simulation.

---

## 10. Next Steps

- Read Hinnen et al. to connect this simulation to the N4CI group's research methodology
- Extend the Kalman state to include actuator dynamics — enables proper lag compensation
- Implement online re-identification via recursive least squares
- Extend to 2D aperture with Zernike basis — true AO geometry

---

*This project was developed as pre-MSc preparation for the Systems and Control program at TU Delft (DCSC), with the goal of building domain expertise in AO-relevant control methods before engaging with the N4CI research group.*
