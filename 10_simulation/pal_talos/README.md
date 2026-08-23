# Evaluated simulation pipeline

Code behind the S1-S5 campaigns reported in the thesis. Snapshot taken at
submission; it is a frozen release, not a live development tree.

## Provenance

The controllers were developed against the TALOS description distributed with
[mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)
(Apache-2.0; `LICENSE` is included here). The robot description files
(`talos.xml` and the `assets/` meshes) are redistributed unmodified under that
licence. Everything else in this directory is original work.

Working layout during development was `mujoco_menagerie/pal_talos/`, i.e. the
controllers sat inside the vendored checkout. They are published here instead
so that they live in this repository rather than inside a third-party clone.
Run the scripts from this directory; the scene files resolve `assets/`
relatively and need no change.

## Layout

- `talos_wbc.py` - the QP-WBC executor used by every scenario
- `talos_dcm_walk*.py` - DCM/LIPM reference generator and step-timing law
- `talos_s3_stairs*.py`, `talos_s4_qs_carry.py`, `talos_s5_perturb.py` - scenario controllers
- `s1_baselines.py` - null-space task-priority and joint-PD baselines
- `centroidal_mpc.py`, `lipm_mpc.py` - the specified planning layer (evaluated offline only)
- `*_batch.py`, `validate_s1_batch.py` - campaign drivers with the Wilson verdict rule
- `probe_*.py`, `qp_*.py`, `s2_determinism.py`, `test_*.py` - audits and probes
- `s*_batch/*_report.md`, `*_rows.csv` - campaign results as reported
- `*_design.md`, `*_diagnosis.md` - design and diagnostic notes

Large batch artefacts (`.npz`), videos and slides are deliberately excluded.

## Requirements

Python 3.13, MuJoCo 3.8, proxsuite, NumPy. `quadprog` is an optional fallback.
