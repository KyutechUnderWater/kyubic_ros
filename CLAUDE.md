# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`kyubic_ros` is the ROS 2 (Jazzy) monorepo for KyutechUnderWater's autonomous underwater vehicles (AUVs): the custom **Kyubic** vehicle and a **BlueROV** (ArduSub) platform. Everything runs inside a Docker container; there is no supported bare-metal ROS install path.

All actual ROS 2 code lives under `kyubic_ws/src/`. The repo root only contains the Docker/install tooling that wraps that workspace.

## Environment: everything happens inside the container

This repo is designed to be cloned once and turned into a long-lived local dev container via `install.sh`, not rebuilt per-session.

```bash
git submodule update --init --recursive   # required: pulls BehaviorTree.CPP and protolink
. install.sh                              # robot computer setup
. install.sh -c                           # client computer setup (adds remote shutdown/ping cmd tools)
. install.sh --nvidia                     # add CUDA support
```

`install.sh` builds the Docker image, creates the container, writes a `ros2_start[_<project>]` shell alias into `~/.bash_aliases`, and stores `KYUBIC_ROS*`/`KYUBIC_ROS_COMPOSE*` env vars in `~/.bashrc`. It refuses to run twice in the same clone (guarded by `.install.project_name`) — use `-p <name>` to stand up multiple parallel environments from separate clones.

Enter the container for day-to-day work:

```bash
ros2_start            # or ros2_start_<project_name> / ros2_start_nvidia if configured
```

Inside the container, `/opt/ros/$ROS_DISTRO/setup.bash` and `kyubic_ws/install/{setup,local_setup}.bash` are auto-sourced via `.bashrc`. The container user is `ros`; Python deps are managed with `uv` (`uv sync`, run automatically by `docker/entrypoint.sh` on start) per `pyproject.toml` / `uv.lock`, Python >=3.12.

## Build and lint (inside the container)

```bash
cd kyubic_ws
colcon build --symlink-install                          # full workspace
colcon build --packages-up-to <pkg1> <pkg2> --symlink-install   # just what you need + deps
source install/setup.bash                                # re-source after any build
```

Rebuild/re-source after changing any package touched by your work — colcon does not auto-source.

Lint/format:
- Python: `ruff` (config in root `pyproject.toml`, line-length 100). Run `ruff check` / `ruff format` from the repo root.
- C++: `ros-jazzy-ament-clang-format` is installed in the image for C++ packages.

Tests: pure-Python `ament_python` packages (e.g. `oak_create_mapping`, `tools/path_generator`, `tools/trdi_toolz`, everything under `visualizer/`) ship the standard `test/test_copyright.py`, `test_flake8.py`, `test_pep257.py` triplet. A handful of C++ packages (`imu_driver`, `dvl75_driver`) have real `ament_add_gtest` unit tests instead (e.g. `test_unit_conversion`, `test_dvl75_parser`). Note `test_copyright.py` is `@pytest.mark.skip`'d repo-wide (no copyright headers in this project) — don't "fix" that by adding headers unless asked.

```bash
colcon test --packages-select <pkg>       # run one package's tests (build it first)
colcon test-result --verbose              # show pass/fail detail after a run
```

## Repository layout

```
kyubic_ws/src/
  common/          shared C++ libs used across drivers/control (custom_socket, serial, timer, pid_controller, geodetic_converter, common_msgs)
  driver/          hardware drivers, split by vehicle
    kyubic/          Kyubic-specific: actuator_rp2040_driver, dvl_driver, gnss_driver, imu_driver, logic_distro_rp2040_driver, sensors_esp32_driver
    blue_rov/        BlueROV-specific: mavlink_driver (Python), dvl75_driver (C++), blue_rov_msgs
    driver_msgs/     vehicle-agnostic common message/service types (IMU, Depth, DVL, PowerState, VehicleState, ...)
    driver_launcher/ per-vehicle launch bundles: kyubic_driver.launch.py / blue_rov_driver.launch.py
  localization/    localization + localization_msgs (Pose, Odometry, GlobalPos, Geodetic, ...)
  control/
    manual/          joy2wrench, joy_common(_msgs) — teleop input -> wrench
    automatic/        planning, emergency
    controller/       p_pid_controller(_msgs)
  behavior_tree/    BT.CPP-based mission logic; bt_xml/ holds mission trees (base.xml, qr.xml, kobe2025.xml, ...)
  kyubic_bringup/   top-level launch entry points (kyubic.launch.py, kyubic_post.launch.py, client*.launch.py, manual.launch.py, web_visualizer.launch.py)
  system_health_check/  monitors node/system health
  visualizer/       dashboard, rt_pose_plotter, trajectory_viewer, web_controller (+ archive/ for retired tools)
  tools/            path_generator, trdi_toolz, plotjuggler helpers
  oak_create_mapping/  OAK-D camera mapping package (ament_python, top-level — doesn't nest under another category)
  sample/           reference/template packages (action server example, BT switch example) — copy from here when scaffolding new packages
  extra/            git submodules: BehaviorTree.CPP, protolink (do not edit; update via submodule)
```

## The cross-vehicle topic contract

This is the architectural rule that lets `localization`, `dashboard`, and other upper layers stay vehicle-agnostic. Read this before touching any driver or launch file.

- **Common topics are vehicle-independent** and must keep the same name and message type regardless of which vehicle is running: `/driver/imu` (`driver_msgs/IMU`), `/driver/depth` (`driver_msgs/Depth`), `/driver/dvl` (`driver_msgs/DVL`). Localization, dashboards, and control consumers subscribe to these directly and should never need vehicle-specific branches.
- **Vehicle-specific topics/services live under a namespace**, e.g. `/driver/blue_rov/mavlink_driver/...`, `/driver/actuator_rp2040_driver/robot_force` for Kyubic. Remapping between vehicles happens in the `driver_launcher` launch files, not in consumer code — e.g. a control node's `robot_force` remap target changes between `/driver/actuator_rp2040_driver/robot_force` (Kyubic) and `/driver/blue_rov/mavlink_driver/robot_force` (BlueROV), and BlueROV additionally requires a `heartbeat` (`std_msgs/Bool`, >=1 Hz) remap that Kyubic control nodes don't need.
- **Device-specific fields go in a separate message**, not into the shared one — e.g. `blue_rov_msgs/DVL75` carries DVL-75-only beam/timing fields so `driver_msgs/DVL` stays common across DVL hardware.
- Only one node may own a given hardware link at a time (e.g. only `mavlink_driver` may hold the MAVLink connection / send RC override — a second concurrent MAVLink client will fight it for control).
- BlueROV's `mavlink_driver` only sends RC override to the vehicle when ALL of: connected, explicitly ARMed via the `set_armed` service (never auto-armed), `robot_force` received within `command_timeout_s` (default 0.5s), and (if `require_control_heartbeat` is set) `heartbeat` received `true` within `control_heartbeat_timeout_s` (default 1.0s). Otherwise it sends neutral PWM. Keep this failsafe behavior in mind when writing or debugging any control node that drives BlueROV.

See `blueNote.md` (Japanese) for a much deeper walkthrough of the BlueROV driver stack (mavlink_driver / dvl75_driver), including full pub/sub tables, config file locations, and manual bring-up/ARM/test procedures — read it before doing nontrivial BlueROV driver work. Per-package READMEs also exist for `driver_launcher`, `mavlink_driver`, `dvl75_driver`, and `blue_rov_msgs`.

## Typical bring-up entry points

```bash
ros2 launch kyubic_bringup kyubic.launch.py          # Kyubic: drivers only (post-launch adds DVL/actuator/localization)
ros2 launch kyubic_bringup kyubic_post.launch.py
ros2 launch driver_launcher kyubic_driver.launch.py     # Kyubic drivers directly
ros2 launch driver_launcher blue_rov_driver.launch.py   # BlueROV: mavlink_driver + dvl75_driver
ros2 launch dvl75_driver dvl75_setup.launch.py          # BlueROV DVL-75 one-time setup only; never run alongside dvl75_driver.launch.py
```

All driver/bringup launch files accept a `log_level` argument (default `info`, bringup-level composition often pins it to `warn`).

## Adding a new package

Use `kyubic_ws/src/sample/` (`action/wrench_action_sample*`, `bt_switch_sample`) as templates rather than starting from scratch — they demonstrate this repo's conventions for action servers and BT node integration. Place new packages under the existing category directory that matches their role (`driver/kyubic` vs `driver/blue_rov`, `control/manual` vs `control/automatic` vs `control/controller`, etc.) rather than inventing a new top-level category.

Never edit code inside `kyubic_ws/src/extra/` (BehaviorTree.CPP, protolink) — those are git submodules; update them via `git submodule update` and upstream them separately if changes are needed.
