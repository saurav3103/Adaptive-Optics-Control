# ── Intensity at focal plane ─────────────────────────────────
def intensity(x, a, c, lam, f):
    u = a * (x - c) / (lam * f)
    return (a**2 / a0**2) * np.sinc(u)**2

# ── Disturbance model ────────────────────────────────────────
A_a  = 0.1e-3   # width jerk amplitude [m]
f_a  = 2.0      # width disturbance frequency [Hz]

A_c  = 0.3e-3   # position jerk amplitude [m]
f_c  = 1.5      # position disturbance frequency [Hz]
phi  = np.pi/4  # phase offset between the two disturbances

noise_std = 0.02e-3  # small additive noise std [m]

def disturbed_slit(t):
    a = a0 + A_a * np.sin(2*np.pi*f_a*t) + np.random.normal(0, noise_std)
    c = c0 + A_c * np.sin(2*np.pi*f_c*t + phi) + np.random.normal(0, noise_std)
    a = float(np.clip(a, 0.05e-3, 2e-3))

    return a, c

# ── Quick sanity check ───────────────────────────────────────
a_test, c_test = disturbed_slit(0.0)
I_test = intensity(x, a_test, c_test, lam, f)
print(f"Test slit: width={a_test*1e3:.3f} mm, center={c_test*1e3:.3f} mm")
print(f"Peak intensity: {I_test.max():.4f}")
print(f"ROI mean intensity: {I_test[roi_mask].mean():.4f}")
