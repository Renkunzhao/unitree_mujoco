# MuJoCo 4.0.0 migration

This branch requires the official MuJoCo 4.0.0 C/C++ distribution. The Python
package is independent of the ROS simulator and is not changed by this migration.

## Build

Extract `mujoco-4.0.0-linux-<architecture>.tar.gz` into `~/.mujoco`, then run
from the workspace root:

```bash
ln -sfnT "$HOME/.mujoco/mujoco-4.0.0" src/unitree_mujoco/simulate/mujoco
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select unitree_mujoco \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
colcon test --packages-select unitree_mujoco
colcon test-result --test-result-base build/unitree_mujoco --verbose
```

The `unitree_lowlevel` branch of the same name updates `scripts/colcon-config.sh`
to download 4.0.0 and replace an existing distribution symlink. A real directory
at `simulate/mujoco` is rejected instead of overwritten. CMake rejects a different
MuJoCo header version, and installation replaces the generated plugin directory
so removed 3.x plugins are not retained.

## Compatibility changes

- Depth cameras use `cam_projection = mjPROJ_PERSPECTIVE` instead of the removed
  `cam_orthographic` field.
- Motor-count logging handles the wider `mjtSize` type.
- The classic OpenGL viewer remains in use. This does not migrate to Studio or
  Filament, enable speculative contacts, or change scene physics parameters.

## Validation on 2026-11-09

On Linux x86_64 with ROS Jazzy:

- Built `unitree_mujoco` and `unitree_lowlevel`; dynamic linking resolves to
  `libmujoco.so.4.0.0` and the runtime reports `4.0.0`.
- All 21 `unitree_robots/*/scene*.xml` scenes load and simulate 1000 steps without
  MuJoCo warnings, time resets, or non-finite positions/velocities. CTest retains
  this check, including Go2 bridge and trampoline scenes.
- ROS checks used loopback only and domain 83. Go2 beam, Go2 trampoline, G1 29-DOF,
  and the Go2 viewer each published at least 100 finite joint-state messages and
  exited cleanly on SIGINT.
- Beam mode published 14 valid 424x240 `16UC1` depth frames, successfully reset
  and started an episode, and resampled payload from 3 to 6 blocks.

These are compatibility smoke tests, not policy performance or hardware tests.
Physics results need not match 3.3.6. The upstream changelog reports a 4.0.0 sparse
Cholesky/Newton regression with a fix listed under the unreleased version at the
time of validation. This branch uses the unmodified official 4.0.0 library and
does not claim to fix that upstream issue.

References: [release](https://github.com/google-deepmind/mujoco/releases/tag/4.0.0),
[changelog](https://mujoco.readthedocs.io/en/latest/changelog.html).
