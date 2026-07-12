# Contributing to Watchpass

Thanks for helping improve Watchpass. This guide covers the development
environment, build/test workflow, and the conventions we follow.

## Development environment

The supported environment is **Ubuntu 24.04 + ROS 2 Jazzy**, provisioned through
Multipass so everyone builds against the same toolchain.

```bash
# Provision the dev VM (see cloud-init/jazzy.yaml for exactly what's installed).
multipass launch 24.04 --name watchpass \
  --cpus 4 --memory 8G --disk 40G \
  --cloud-init cloud-init/jazzy.yaml

# Share your checkout into the VM and open a shell.
multipass mount . watchpass:/home/ubuntu/watchpass
multipass shell watchpass
cd ~/watchpass
```

`/etc/profile.d/99-watchpass.sh` (installed by cloud-init) sources ROS, selects
`rmw_fastrtps_cpp`, and exports the shared-memory Fast DDS profile, so a fresh
login shell is ready to build.

Working natively instead? Install `ros-jazzy-desktop`, `ros-jazzy-ros-gz`,
`ros-jazzy-rmw-fastrtps-cpp`, and `ffmpeg`, then `source /opt/ros/jazzy/setup.bash`.

## Build & test

```bash
# Build everything.
colcon build
source install/setup.bash

# Iterate on one package.
colcon build --packages-select watchpass_streamer

# Run tests + linters (ament_lint runs during test).
colcon test
colcon test-result --verbose
```

`watchpass_msgs` must build before the packages that depend on it — `colcon`
orders this for you as long as you build from the workspace root.

## Verifying a change

Beyond `colcon test`, exercise the runtime paths your change touches:

- **Zero-copy loan path** — `ros2 run watchpass_streamer camera_bridge` and
  confirm the startup log says `loaned messages ENABLED (zero-copy)`.
- **Bridge conversion** — publish a `sensor_msgs/Image` and check a `WatchFrame`
  appears with a matching `data_size` (`ros2 topic echo /watch_frame --field data_size`).
- **Encoder pipe** — set `ffmpeg_path` to a script that logs its argv and stdin
  byte count to confirm the frames and geometry reach ffmpeg correctly.
- **End to end** — `ros2 launch watchpass_bringup sim.launch.py` and view the
  stream at `http://<vm-ip>:8889/watchpass`.

## Coding conventions

- **C++**: C++17, follow the ROS 2 style enforced by `ament_lint`/`ament_uncrustify`
  (4-space indent, 100-column soft limit). Every source file carries the
  `SPDX-License-Identifier: Apache-2.0` header.
- **Messages**: keep interfaces in `watchpass_msgs` **plain** (fixed-size fields
  only). Adding a string or unbounded array silently disables the zero-copy loan
  path — if you must, document the trade-off. If you change `MAX_DATA`, update
  both the constant and the `uint8[...]` array bound, then rebuild all packages.
- **Launch/Python**: follow PEP 8; `ament_flake8` runs in `colcon test`.
- **Parameters over hard-coding**: expose new behaviour as a node parameter with
  a sensible default, and add it to the table in the README.

## Commit & PR guidelines

- Use focused commits with imperative subject lines (e.g. `streamer: restart
  ffmpeg on geometry change`).
- A PR should build cleanly (`colcon build`), pass `colcon test`, and describe
  how you verified the change (which of the checks above you ran).
- Keep documentation in step with behaviour — update the README config table and
  this guide when you add parameters or change the setup.

## Project layout

```
watchpass/
├── cloud-init/             # Multipass provisioning (jazzy.yaml, mediamtx.yml)
├── docs/                   # plan.md, design_generic_transport.md
└── src/
    ├── watchpass_msgs/      # WatchFrame + WatchFrameHandle (rosidl); capacity knob
    ├── watchpass_framework/ # FrameView + FrameConsumerNode + FrameProcessorNode
    ├── watchpass_nodes/     # example processors: grayscale, flip
    ├── watchpass_streamer/  # camera_bridge + ffmpeg_streamer (consumer), FfmpegPipe
    └── watchpass_bringup/   # launch files, DDS SHM profile, demo world
```

### Adding a processing node

Subclass `watchpass::FrameProcessorNode` (in→out) or `FrameConsumerNode` (sink),
implement the one virtual method, and register it with
`RCLCPP_COMPONENTS_REGISTER_NODE`. Use `FrameView::as_mat()` for zero-copy OpenCV
access and write results into the loaned output buffer via a `cv::Mat` that
aliases `out.data.data()`. See `watchpass_nodes` for worked examples and the
README's "Writing your own processing node".

Keep encoding logic in `FrameFormat` (in `watchpass_framework`) so every node
agrees on the encoding ↔ OpenCV ↔ ffmpeg mappings.

By contributing you agree that your work is licensed under the project's
[Apache-2.0](LICENSE) license.
