# Repository Guidelines

## Project Structure & Module Organization

- `src/peach/`: perception, reconstruction, scheduling, arm execution, bringup, observability, and Web packages. Cross-package contracts live in `peach_interfaces`; communicate through ROS interfaces, not business-module imports.
- `src/ros2_web_link/`: shared browser SDK; Peach assets are in `src/peach/peach_web/web/`.
- Models: `src/peach/peach_scene_perception/model/`; robot assets: `src/aubo_description/`.
- Package `doc/architecture.md` and `doc/io.md` describe current behavior. Update them alongside implementation changes.
- Manual checks: `src/peach/peach_system_tests/scripts/`; evidence: `runs/`. Exclude `build/`, `install/`, `log/`, and `_archive/` from commits.

## Build, Test, and Development Commands

Run from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to peach_system_tests peach_web peach_stereo peach_vegetation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
source aubo_py3.12/bin/activate
export PYTHONPATH="$PWD/aubo_py3.12/lib/python3.12/site-packages:$PYTHONPATH"
colcon test --packages-select peach_arm peach_supervisor --return-code-on-test-failure
colcon test-result --verbose
npm --prefix src/ros2_web_link run check
ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock \
  camera_enabled:=false force_enabled:=false rviz:=false autostart:=false
```

These build dependencies, check selected packages and SDK types, and start hardware-free development. Replace the test selection with affected packages.

## Coding Style & Naming Conventions

Use Jazzy C++17 with two-space indentation and `ament_uncrustify`; Python uses four spaces, PEP 8, type annotations, `ament_flake8`, and `ament_pep257`. Package/module names use `snake_case`; launch files use `*.launch.py`. Consult Jazzy documentation and matching API source before implementation. Preserve joint ordering and existing interface contracts.

## Testing Guidelines

Peach automated tests contain official ament lint only: pytest wrappers `test_flake8.py`/`test_pep257.py`, plus CMake lint including cppcheck/xmllint. Do not add custom behavioral suites under `test/`. There is no numeric coverage gate; lint does not establish harvest correctness.

Follow [testing.md](src/peach/peach_system_tests/doc/testing.md) for mock/replay checks. After interface changes, run `python3 src/peach/peach_interfaces/scripts/check_interface_manifest.py`.

## Commit & Pull Request Guidelines

History uses `feat:`, `fix:`, `refactor:`, and `chore:`, optionally scoped, e.g. `fix(arm): handle cancellation`. Describe behavior, affected packages, validation results, relevant issues, and Web screenshots when applicable. Preserve unrelated worktree changes.

## Safety & Agent Instructions

Respond in Chinese. Keep driver hardware/controllers, dashboard, ros2_control xacro, driver bringup, and controller configuration read-only. Never launch/call dashboard. Real motion or SetIO requires operator authorization; hardware emergency stops bypass ROS. Keep enables/autostart off by default. Gazebo is suspended unless explicitly restored; mock/replay remain available. Use Jazzy apt dependencies and `aubo_py3.12`; pin `numpy==1.26.4`. Inventory processes before and after every run; stop all owned launches, probes, and recordings without killing unrelated tasks.
