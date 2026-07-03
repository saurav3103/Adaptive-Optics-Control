# ── Block 5: Static System Identification ────────────────────

# ── Two ROI zones ────────────────────────────────────────────
roi_L = (x >= -0.5e-3) & (x < 0.0)
roi_R = (x >= 0.0)     & (x <= 0.5e-3)

def measure(a, c):
    """Return measurement vector s = [s_L, s_R]"""
    I  = intensity(x, a, c, lam, f)
    sL = I[roi_L].mean()
    sR = I[roi_R].mean()
    return np.array([sL, sR])

# ── Nominal measurement ───────────────────────────────────────
s_nom = measure(a0, c0)
print(f"Nominal measurement: sL={s_nom[0]:.4f}, sR={s_nom[1]:.4f}")

# ── Poke amplitudes ───────────────────────────────────────────
d_c = 0.03e-3    # 10% of disturbance amplitude
d_a = 0.05e-3    # 10% of nominal width

# ── Poke actuator 1: slit center c ───────────────────────────
s_c_pos = measure(a0, c0 + d_c)
s_c_neg = measure(a0, c0 - d_c)
col_c   = (s_c_pos - s_c_neg) / (2 * d_c)   # finite difference
print(f"\nPoke c: s_pos={s_c_pos}, s_neg={s_c_neg}")
print(f"Column 1 of D: {col_c}")

# ── Poke actuator 2: slit width a ────────────────────────────
s_a_pos = measure(a0 + d_a, c0)
s_a_neg = measure(a0 - d_a, c0)
col_a   = (s_a_pos - s_a_neg) / (2 * d_a)   # finite difference
print(f"\nPoke a: s_pos={s_a_pos}, s_neg={s_a_neg}")
print(f"Column 2 of D: {col_a}")

# ── Build interaction matrix D ────────────────────────────────
D = np.column_stack([col_c, col_a])
print(f"\nInteraction matrix D:")
print(D)
print(f"\nCondition number of D: {np.linalg.cond(D):.2f}")

# ── Compute control matrix D+ (pseudoinverse) ─────────────────
D_plus = np.linalg.pinv(D)
print(f"\nControl matrix D+:")
print(D_plus)

# ── Validate: predict response to a known perturbation ────────
test_dc = 0.1e-3
test_da = 0.05e-3
s_test_actual    = measure(a0 + test_da, c0 + test_dc)
ds_test_actual   = s_test_actual - s_nom
ds_test_predicted = D @ np.array([test_dc, test_da])

print(f"\nValidation:")
print(f"Actual    ds: {ds_test_actual}")
print(f"Predicted ds: {ds_test_predicted}")
print(f"Error:        {np.abs(ds_test_actual - ds_test_predicted)}")
test_dc = 0.01e-3
test_da = 0.01e-3
s_test_actual     = measure(a0 + test_da, c0 + test_dc)
ds_test_actual    = s_test_actual - s_nom
ds_test_predicted = D @ np.array([test_dc, test_da])

print(f"Actual    ds: {ds_test_actual}")
print(f"Predicted ds: {ds_test_predicted}")
print(f"Relative error: {np.abs(ds_test_actual - ds_test_predicted)/np.abs(ds_test_actual)*100} %")

# ── Block 6: D+ Controller ───────────────────────────────────
np.random.seed(42)

a_dp_log   = np.zeros(len(time))
c_dp_log   = np.zeros(len(time))
c_act_log  = np.zeros(len(time))
roi_dp_log = np.zeros(len(time))

for i, t in enumerate(time):
    # step 1: disturbance moves the slit
    a_dist, c_dist = disturbed_slit(t)

    # step 2: measure current sensor output
    s_measured = measure(a_dist, c_dist)

    # step 3: compute sensor deviation from nominal
    ds = s_measured - s_nom

    # step 4: compute actuator correction via D+
    # u_cmd = [delta_c, delta_a]
    u_cmd = -D_plus @ ds

    # step 5: apply correction
    c_act = c_dist + u_cmd[0]
    a_act = np.clip(a_dist + u_cmd[1], 0.05e-3, 2e-3)

    # step 6: measure corrected intensity
    I = intensity(x, a_act, c_act, lam, f)

    a_dp_log[i]   = a_act
    c_dp_log[i]   = c_dist
    c_act_log[i]  = c_act
    roi_dp_log[i] = I[roi_mask].mean()

# ── Plot ─────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 1, figsize=(10, 6), tight_layout=True)

axes[0].plot(time, roi_log,    color='crimson',   alpha=0.7,
             label=f'Open loop   (mean={roi_log.mean():.3f}, std={roi_log.std():.3f})')
axes[0].plot(time, roi_cl_log, color='steelblue', alpha=0.7,
             label=f'PI control  (mean={roi_cl_log.mean():.3f}, std={roi_cl_log.std():.3f})')
axes[0].plot(time, roi_dp_log, color='green',     alpha=0.8,
             label=f'D+ control  (mean={roi_dp_log.mean():.3f}, std={roi_dp_log.std():.3f})')
axes[0].set_ylabel('ROI mean intensity')
axes[0].set_xlabel('Time [s]')
axes[0].legend()
axes[0].set_title('ROI Intensity: Open Loop vs PI vs D+ Controller')

axes[1].plot(time, c_dp_log*1e3,  color='darkorange', alpha=0.9, label='Disturbance c(t)')
axes[1].plot(time, c_act_log*1e3, color='green',      alpha=0.9, label='Actuated c(t)')
axes[1].axhline(0, color='k', linestyle='--', linewidth=0.8, label='optimal')
axes[1].set_ylabel('Slit center [mm]')
axes[1].set_xlabel('Time [s]')
axes[1].legend()

plt.savefig('dplus_control.png', dpi=150)
plt.show()

print(f"Open loop   — mean: {roi_log.mean():.4f}, std: {roi_log.std():.4f}")
print(f"PI control  — mean: {roi_cl_log.mean():.4f}, std: {roi_cl_log.std():.4f}")
print(f"D+ control  — mean: {roi_dp_log.mean():.4f}, std: {roi_dp_log.std():.4f}")
