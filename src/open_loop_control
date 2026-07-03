# ── Open loop simulation ─────────────────────────────────────
np.random.seed(42)

a_log     = np.zeros(len(time))
c_log     = np.zeros(len(time))
roi_log   = np.zeros(len(time))

for i, t in enumerate(time):
    a, c       = disturbed_slit(t)
    I          = intensity(x, a, c, lam, f)
    a_log[i]   = a
    c_log[i]   = c
    roi_log[i] = I[roi_mask].mean()

# ── Plot ─────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(10, 8), tight_layout=True)

axes[0].plot(time, a_log*1e3, color='steelblue')
axes[0].axhline(a0*1e3, color='k', linestyle='--', linewidth=0.8, label='nominal')
axes[0].set_ylabel('Slit width [mm]')
axes[0].legend()

axes[1].plot(time, c_log*1e3, color='darkorange')
axes[1].axhline(0, color='k', linestyle='--', linewidth=0.8, label='nominal')
axes[1].set_ylabel('Slit center [mm]')
axes[1].legend()

axes[2].plot(time, roi_log, color='crimson')
axes[2].axhline(roi_log.mean(), color='k', linestyle='--', linewidth=0.8, label=f'mean={roi_log.mean():.3f}')
axes[2].set_ylabel('ROI mean intensity')
axes[2].set_xlabel('Time [s]')
axes[2].legend()

plt.suptitle('Open Loop — Disturbed Slit', fontweight='bold')
plt.savefig('open_loop.png', dpi=150)
plt.show()
print(f"ROI mean: {roi_log.mean():.4f}, min: {roi_log.min():.4f}, max: {roi_log.max():.4f}")
