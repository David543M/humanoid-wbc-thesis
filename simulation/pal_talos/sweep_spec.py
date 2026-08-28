"""Timing/tracking sweep on the SPECIFIED horizon band (Ch3 3.3: N.dt in [1.0,2.0] s, dt=50 ms)."""
import numpy as np
from test_centroidal_mpc import run

print("%-5s %-7s %-9s %-8s %-9s %-10s %-9s %s"
      % ("N", "dt", "horiz s", "n_var", "mean ms", "p99 ms", "RMSE mm", "statuses"))
for N, dtm in ((8,0.05),(10,0.05),(12,0.05),(20,0.05),(30,0.05),(40,0.05)):
    r = run(horizon=N, dt_mpc=dtm, duration=3.0, verbose=False)
    m = r["mpc"]; t = np.array(m.stats["t_ms"])
    print("%-5d %-7.3f %-9.2f %-8d %-9.2f %-10.2f %-9.1f %s"
          % (N, dtm, N*dtm, N*m.nu, t.mean(), np.percentile(t,99),
             r["rmse_mm"], m.stats["statuses"]))
