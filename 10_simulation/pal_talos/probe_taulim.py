"""Quantify: the QP's torque bounds are the +/-300 fallback, not the model's ctrlrange.
Measures how often the QP-commanded torque is outside what the plant will actually apply."""
import numpy as np, mujoco, talos_wbc as W

m = mujoco.MjModel.from_xml_path(W.MODEL); d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
wbc = W.WBC(m, d, corners=W.CORNERS_MEASURED)

print("QP torque bounds actually used by the executor:")
print("  tau_min unique:", np.unique(wbc.tau_min))
print("  tau_max unique:", np.unique(wbc.tau_max))
print("model ctrlrange (what the plant enforces):")
print("  lower unique:", np.unique(m.actuator_ctrlrange[:,0]))
print("  upper unique:", np.unique(m.actuator_ctrlrange[:,1]))
tighter = m.actuator_ctrlrange[:,1] < wbc.tau_max
print("  actuators where the PLANT limit is TIGHTER than the QP bound: %d/%d" % (tighter.sum(), m.nu))
looser  = m.actuator_ctrlrange[:,1] > wbc.tau_max
print("  actuators where the PLANT limit is LOOSER  than the QP bound: %d/%d" % (looser.sum(), m.nu))
print()

lo, hi = m.actuator_ctrlrange[:,0], m.actuator_ctrlrange[:,1]
n_tick = 0; n_clip_tick = 0; clip_per_act = np.zeros(m.nu, int)
max_excess = np.zeros(m.nu)
AXIS, FORCE, T_PUSH, IMP = 1, 150.0, 2.0, 0.1
while d.time < 6.0:
    tau = wbc.control()            # returns the (already clipped-to-+/-300) command
    d.xfrc_applied[wbc.base, :] = 0.0
    if T_PUSH <= d.time < T_PUSH + IMP:
        d.xfrc_applied[wbc.base, AXIS] = FORCE
    over = (tau < lo - 1e-9) | (tau > hi + 1e-9)
    n_tick += 1
    if over.any():
        n_clip_tick += 1
        clip_per_act += over
        ex = np.maximum(tau - hi, lo - tau)
        max_excess = np.maximum(max_excess, np.where(over, ex, 0.0))
    mujoco.mj_step(m, d)

print("ticks simulated: %d (6 s at 1 kHz)" % n_tick)
print("ticks where >=1 QP torque exceeds the plant's ctrlrange: %d (%.1f %%)"
      % (n_clip_tick, 100.0*n_clip_tick/n_tick))
print()
print("per-actuator (only those that ever exceeded):")
for i in range(m.nu):
    if clip_per_act[i]:
        print("  %-28s plant limit +/-%6.1f | QP bound +/-%6.1f | exceeded %5d ticks (%.1f %%) | worst excess %7.1f N.m"
              % (m.actuator(i).name.replace('_torque',''), hi[i], wbc.tau_max[i],
                 clip_per_act[i], 100.0*clip_per_act[i]/n_tick, max_excess[i]))
if clip_per_act.sum() == 0:
    print("  none — the QP never commanded a torque the plant had to clip in this run.")
print()
print("QP stats:", W.qp_stats_report())
