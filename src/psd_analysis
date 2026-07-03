from scipy import signal as sig

# ── Block 8: Power Spectral Density Analysis ─────────────────

# recompute clean disturbance signal
np.random.seed(42)
c_dist_log_clean = np.zeros(len(time))
a_dist_log_clean = np.zeros(len(time))

for i, t in enumerate(time):
    a_d, c_d = disturbed_slit(t)
    c_dist_log_clean[i] = c_d
    a_dist_log_clean[i] = a_d

# ── Welch PSD ────────────────────────────────────────────────
fs       = 1/dt        # sampling frequency = 1000 Hz
nperseg  = 8192        # df = 1000/8192 = 0.122Hz
noverlap = 4096        # 50% overlap

f_c, psd_c = sig.welch(c_dist_log_clean, fs=fs, nperseg=nperseg, noverlap=noverlap)
f_a, psd_a = sig.welch(a_dist_log_clean, fs=fs, nperseg=nperseg, noverlap=noverlap)

print(f"Frequency resolution: {fs/nperseg:.3f} Hz")
print(f"Number of windows:    {(len(c_dist_log_clean) - nperseg)//(nperseg - noverlap)}")

# ── Plot ─────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 1, figsize=(10, 7), tight_layout=True)

axes[0].semilogy(f_c, psd_c, color='darkorange')
axes[0].axvline(1.5, color='k', linestyle='--', linewidth=0.8, label='1.5 Hz')
axes[0].axvline(2.0, color='r', linestyle='--', linewidth=0.8, label='2.0 Hz')
axes[0].set_xlim(0, 10)
axes[0].set_xlabel('Frequency [Hz]')
axes[0].set_ylabel('PSD [m²/Hz]')
axes[0].set_title('PSD of Slit Center Disturbance c(t)')
axes[0].legend()

axes[1].semilogy(f_a, psd_a, color='steelblue')
axes[1].axvline(2.0, color='k', linestyle='--', linewidth=0.8, label='2.0 Hz')
axes[1].set_xlim(0, 10)
axes[1].set_xlabel('Frequency [Hz]')
axes[1].set_ylabel('PSD [m²/Hz]')
axes[1].set_title('PSD of Slit Width Disturbance a(t)')
axes[1].legend()

plt.savefig('psd_analysis.png', dpi=150)
plt.show()

# ── Find peaks ───────────────────────────────────────────────
peaks_c, props_c = sig.find_peaks(psd_c, height=1e-12)
peaks_a, props_a = sig.find_peaks(psd_a, height=1e-13)

print("\nTop 3 peaks in c(t) PSD:")
top3_c = np.argsort(props_c['peak_heights'])[-3:][::-1]
for idx in top3_c:
    print(f"  f={f_c[peaks_c[idx]]:.3f} Hz, PSD={props_c['peak_heights'][idx]:.2e}")

print("\nTop 3 peaks in a(t) PSD:")
top3_a = np.argsort(props_a['peak_heights'])[-3:][::-1]
for idx in top3_a:
    print(f"  f={f_a[peaks_a[idx]]:.3f} Hz, PSD={props_a['peak_heights'][idx]:.2e}")
