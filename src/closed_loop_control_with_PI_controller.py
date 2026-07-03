class PID:
    def __init__(self, Kp, Ki, Kd, dt, output_min, output_max):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.output_min = output_min
        self.output_max = output_max
        self.integral   = 0.0
        self.prev_error = 0.0

    def step(self, error):
        p = self.Kp * error
        d = self.Kd * (error - self.prev_error) / self.dt
        self.prev_error = error

        output_pd = p + d
        output_unclipped = output_pd + self.Ki * self.integral
        # anti-windup: only integrate if not saturated
        if not (output_unclipped >= self.output_max and error > 0) and \
           not (output_unclipped <= self.output_min and error < 0):
            self.integral += error * self.dt

        output = output_pd + self.Ki * self.integral
        return np.clip(output, self.output_min, self.output_max)
np.random.seed(42)

pid_c = PID(Kp=0.8, Ki=0.1, Kd=0.0, dt=dt,
            output_min=-0.4e-3, output_max=0.4e-3)
pid_a = PID(Kp=0.8, Ki=0.1, Kd=0.0, dt=dt,
            output_min=-0.3e-3, output_max=0.3e-3)

a_cl_log   = np.zeros(len(time))
c_cl_log   = np.zeros(len(time))
c_act_log  = np.zeros(len(time))
roi_cl_log = np.zeros(len(time))

for i, t in enumerate(time):
    a_dist, c_dist = disturbed_slit(t)

    e_c = 0.0 - c_dist
    e_a = a0  - a_dist

    u_c = pid_c.step(e_c)
    u_a = pid_a.step(e_a)

    c_act = c_dist + u_c
    a_act = np.clip(a_dist + u_a, 0.05e-3, 2e-3)

    I = intensity(x, a_act, c_act, lam, f)

    a_cl_log[i]   = a_act
    c_cl_log[i]   = c_dist
    c_act_log[i]  = c_act
    roi_cl_log[i] = I[roi_mask].mean()

fig, axes = plt.subplots(2, 1, figsize=(10, 6), tight_layout=True)

axes[0].plot(time, roi_log,    color='crimson',   alpha=0.8,
             label=f'Open loop   (mean={roi_log.mean():.3f}, std={roi_log.std():.3f})')
axes[0].plot(time, roi_cl_log, color='steelblue', alpha=0.8,
             label=f'Closed loop (mean={roi_cl_log.mean():.3f}, std={roi_cl_log.std():.3f})')
axes[0].set_ylabel('ROI mean intensity')
axes[0].set_xlabel('Time [s]')
axes[0].legend()
axes[0].set_title('ROI Intensity: Open vs Closed Loop (PI)')

axes[1].plot(time, c_cl_log*1e3,  color='darkorange', alpha=0.9, label='Disturbance c(t)')
axes[1].plot(time, c_act_log*1e3, color='steelblue',  alpha=0.9, label='Actuated c(t)')
axes[1].axhline(0, color='k', linestyle='--', linewidth=0.8, label='optimal')
axes[1].set_ylabel('Slit center [mm]')
axes[1].set_xlabel('Time [s]')
axes[1].legend()

plt.savefig('closed_loop_PI.png', dpi=150)
plt.show()

print(f"Open loop   — mean: {roi_log.mean():.4f}, std: {roi_log.std():.4f}")
print(f"Closed loop — mean: {roi_cl_log.mean():.4f}, std: {roi_cl_log.std():.4f}")
