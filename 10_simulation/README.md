# 10_simulation — Simulation Assets
*Last updated: 2026-05-03*

All simulation assets for the thesis experiments. Subfolders:

| Folder | Contents | Status |
|--------|----------|--------|
| `urdf_sdf_models/` | TALOS URDF (PAL Robotics) + MJCF conversion | 🔲 To populate |
| `scenes/` | MuJoCo XML scene files (flat ground, stairs, obstacles) | 🔲 To populate |
| `configs/` | YAML/JSON configs: QP solver, task hierarchy, MPC horizon, scenario params | 🔲 To populate |
| `controllers_src/` | Live controller source (Python/C++); frozen snapshots → `09_appendices/code/` | 🔲 To populate |

## Planned Software Stack

| Component | Tool | Version |
|-----------|------|---------|
| Simulator | MuJoCo | ≥ 3.x |
| Kinematics / dynamics | Pinocchio | ≥ 2.7 |
| QP solver | Eiquadprog or PROXQP | — |
| MPC | Custom centroidal MPC (Python) | — |
| Interface | Python bindings (mujoco-py or dm_control) | — |

## TALOS model notes
- DOF: 32 (6 unactuated floating base + 26 actuated joints)
- Torque-controlled joints — ideal actuator model in simulation
- URDF source: PAL Robotics open-source repository
- Inertial parameters: to be validated against published TALOS specs before Ch4

## Setup (to complete in Ch4 phase)
```bash
pip install mujoco pinocchio
# Clone TALOS URDF
git clone https://github.com/pal-robotics/talos_robot
# Convert to MJCF
python scripts/urdf_to_mjcf.py
```
