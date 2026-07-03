from scipy.linalg import expm

# ── Block 9: LQG Controller ───────────────────────────────────

# ── Disturbance frequencies from PSD ─────────────────────────
wc = 2*np.pi*1.5    # center disturbance [rad/s]
wa = 2*np.pi*2.0    # width disturbance [rad/s]

# ── Continuous time A and C matrices ─────────────────────────
A_ct = np.array([[0,      1,  0,      0],
                 [-wc**2, 0,  0,      0],
                 [0,      0,  0,      1],
                 [0,      0, -wa**2,  0]])

C = np.array([[1, 0, 0, 0],
              [0, 0, 1, 0]])

# ── Discretize A ─────────────────────────────────────────────
A_d = expm(A_ct * dt)

# ── Noise covariance matrices ─────────────────────────────────
Q = np.diag([1e-6, 1e-6, 1e-8, 1e-8])
R = np.diag([1e-8, 1e-8])

# ── Steady state Kalman gain (iterative Riccati) ──────────────
P = np.eye(4) * 1e-6
for _ in range(2000):
    P_pred = A_d @ P @ A_d.T + Q
    S      = C @ P_pred @ C.T + R
    K_kal  = P_pred @ C.T @ np.linalg.inv(S)
    P      = (np.eye(4) - K_kal @ C) @ P_pred

print("Kalman gain K_kal:")
print(np.round(K_kal, 6))

# ── Simulation ────────────────────────────────────────────────
np.random.seed(42)

c_actuator = 0.0
a_actuator = 0.0
x_hat      = np.zeros(4)

a_lqg_log   = np.zeros(len(time))
c_lqg_log   = np.zeros(len(time))
c_act_log   = np.zeros(len(time))
roi_lqg_log = np.zeros(len(time))

for i, t in enumerate(time):
    # step 1: disturbance moves the slit
    a_dist, c_dist = disturbed_slit(t)

    # step 2: measurement
    y = np.array([c_dist, a_dist])

    # step 3: Kalman predict + update
    y_pred = C @ x_hat
    x_hat  = A_d @ x_hat + K_kal @ (y - y_pred)

    # step 4: feedforward — negate estimated disturbance
    u_c = -x_hat[0]           # cancel estimated center disturbance
    u_a = -(x_hat[2] - a0)    # cancel estimated width disturbance

    # step 5: actuator lag
    c_actuator = alpha_c * c_actuator + (1 - alpha_c) * u_c
    a_actuator = alpha_a * a_actuator + (1 - alpha_a) * u_a

    # step 6: apply correction
    c_act = c_dist + c_actuator
    a_act = np.clip(a_dist + a_actuator, 0.05e-3, 2e-3)

    # step 7: measure intensity
    I = intensity(x, a_act, c_act, lam, f)

    a_lqg_log[i]   = a_act
    c_lqg_log[i]   = c_dist
    c_act_log[i]   = c_act
    roi_lqg_log[i] = I[roi_mask].mean()

# ── Plot ─────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 1, figsize=(10, 6), tight_layout=True)

axes[0].plot(time, roi_log,     color='crimson', alpha=0.6,
             label=f'Open loop   (mean={roi_log.mean():.3f}, std={roi_log.std():.3f})')
axes[0].plot(time, roi_lag_log, color='purple',  alpha=0.6,
             label=f'D+ with lag (mean={roi_lag_log.mean():.3f}, std={roi_lag_log.std():.3f})')
axes[0].plot(time, roi_lqg_log, color='green',   alpha=0.8,
             label=f'LQG         (mean={roi_lqg_log.mean():.3f}, std={roi_lqg_log.std():.3f})')
axes[0].set_ylabel('ROI mean intensity')
axes[0].set_xlabel('Time [s]')
axes[0].legend()
axes[0].set_title('ROI Intensity: Open Loop vs D+ with lag vs LQG')

axes[1].plot(time, c_lqg_log*1e3, color='darkorange', alpha=0.9, label='Disturbance c(t)')
axes[1].plot(time, c_act_log*1e3, color='green',      alpha=0.9, label='Actuated c(t) LQG')
axes[1].axhline(0, color='k', linestyle='--', linewidth=0.8, label='optimal')
axes[1].set_ylabel('Slit center [mm]')
axes[1].set_xlabel('Time [s]')
axes[1].legend()

plt.savefig('lqg_control.png', dpi=150)
plt.show()

print(f"Open loop   — mean: {roi_log.mean():.4f}, std: {roi_log.std():.4f}")
print(f"D+ with lag — mean: {roi_lag_log.mean():.4f}, std: {roi_lag_log.std():.4f}")
print(f"LQG         — mean: {roi_lqg_log.mean():.4f}, std: {roi_lqg_log.std():.4f}")
