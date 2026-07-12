# Watchpass

A zero-copy image pipeline **framework** for ROS 2 Jazzy. Ingest frames from a
camera or image driver into shared-memory ROS buffers, chain any number of
image-processing nodes over them without copies, and encode + ship the result to
a WebRTC/RTSP/RTP server with `ffmpeg` — including AMD/Intel GPU (VA-API) encode.

```
 camera / Gazebo        camera_bridge        FrameProcessorNode(s)       ffmpeg_streamer        mediamtx
 (sensor_msgs/Image) ─▶ (loan a WatchFrame)─▶ grayscale ─▶ flip ─▶ ... ─▶ (encode) ──RTSP──▶ (WebRTC :8889)
                        └───────────── zero-copy loaned WatchFrame ──────┘ └─ kernel pipe ─┘
```

`WatchFrame` is the common currency. `ffmpeg_streamer` is just one *consumer* of
it; every processing stage is a small subclass of a framework base class and
gets zero-copy in **and** out for free.

- **Zero-copy ingest into ROS.** A fixed-size `WatchFrame` (generated with
  `rosidl`) is a *plain* type, so Fast DDS hands publishers and subscribers a
  **loaned buffer** in shared memory — no serialization, no copies between nodes.
- **A processing framework, not one app.** `FrameProcessorNode` (in→out) and
  `FrameConsumerNode` (sink) own all the plumbing; you implement one method.
  `FrameView::as_mat()` gives a **zero-copy `cv::Mat`** for OpenCV work.
- **ffmpeg on a ROS node.** `ffmpeg_streamer` forks `ffmpeg`, feeds it raw frames
  over a **pipe**, and pushes to any `ffmpeg` output (RTSP→WebRTC via mediamtx,
  RTP, WHIP). Software (`x264`) or **AMD/Intel GPU (`vaapi`)** encode.
- **Configurable, not hardcoded.** Frame capacity is a build-time knob
  (`WATCHPASS_FRAME_CAPACITY`); a `WatchFrameHandle` type is defined for the
  4K/GPU-DMABUF path. See [docs/design_generic_transport.md](docs/design_generic_transport.md).
- **Reproducible setup.** A Multipass `cloud-init` provisions the whole Jazzy +
  Gazebo + ffmpeg + VA-API toolchain (see [Setup](#setup)).

## Packages

| Package | Purpose |
| --- | --- |
| [`watchpass_msgs`](src/watchpass_msgs) | `WatchFrame` (fixed-size, zero-copy) + `WatchFrameHandle` (reference/DMABUF), via `rosidl`. Capacity is a CMake knob. |
| [`watchpass_framework`](src/watchpass_framework) | The reusable core: `FrameView` (OpenCV zero-copy), `FrameConsumerNode`, `FrameProcessorNode`. |
| [`watchpass_nodes`](src/watchpass_nodes) | Example processors on the framework: `grayscale`, `flip`. |
| [`watchpass_streamer`](src/watchpass_streamer) | `camera_bridge` (ingest) + `ffmpeg_streamer` (a `FrameConsumerNode`) + `FfmpegPipe`. |
| [`watchpass_bringup`](src/watchpass_bringup) | Launch files, Fast DDS shared-memory profile, demo Gazebo world. |

## Why this is zero-copy

ROS 2 delivers a message with no copy when three things line up, and Watchpass
arranges all three:

1. **A plain message type.** `WatchFrame` has only fixed-size fields (a bounded
   `uint8[N]` payload sized by `WATCHPASS_FRAME_CAPACITY`, no strings or
   unbounded arrays). Variable-length fields would force serialize-and-copy.
2. **Shared-memory transport with data-sharing.** The Fast DDS profile in
   [`fastdds_shm.xml`](src/watchpass_bringup/config/fastdds_shm.xml) enables the
   SHM transport and `AUTOMATIC` data-sharing with a `PREALLOCATED` history.
3. **Loaned messages.** `camera_bridge` publishes via
   `borrow_loaned_message()` — writing straight into the shared buffer the
   subscriber will read. At startup it logs whether the loan path is active:

   ```
   camera_bridge: image_raw -> watch_frame, loaned messages ENABLED (zero-copy)
   ```

Run both nodes in one **component container** (as `stream.launch.py` does) and
intra-process comms make the hand-off a pointer move even without SHM. From the
streamer, frames reach `ffmpeg` over a pipe — the one place bytes are handed to
another program.

> **Real cameras / DMA.** In simulation frames arrive as `sensor_msgs/Image`, so
> `camera_bridge` does a single copy into the loaned buffer. A real driver would
> eliminate that too by filling the loaned `WatchFrame` directly from its
> `V4L2`/DMABUF capture buffer — the pattern is identical, only the source of the
> pixels changes.

## Setup

Everything is pinned by a Multipass `cloud-init`. You don't have to boot the VM
to read the setup — [`cloud-init/jazzy.yaml`](cloud-init/jazzy.yaml) *is* the
contract — but to get a working environment:

```bash
# 1. Provision an Ubuntu 24.04 VM with Jazzy + Gazebo + ffmpeg + mediamtx.
multipass launch 24.04 --name watchpass \
  --cpus 4 --memory 8G --disk 40G \
  --cloud-init cloud-init/jazzy.yaml

# 2. Share this checkout into the VM and open a shell.
multipass mount . watchpass:/home/ubuntu/watchpass
multipass shell watchpass

# 3. Build (ROS is already sourced by /etc/profile.d/99-watchpass.sh).
cd ~/watchpass
colcon build
source install/setup.bash
```

Prefer a native machine? On Ubuntu 24.04 with `ros-jazzy-desktop`,
`ros-jazzy-ros-gz`, and `ffmpeg` installed, steps 3 onward are identical.

## Run

### Full simulation demo

```bash
# Terminal A — WebRTC server (installed by cloud-init).
mediamtx cloud-init/mediamtx.yml

# Terminal B — Gazebo camera -> zero-copy pipeline -> ffmpeg -> RTSP.
ros2 launch watchpass_bringup sim.launch.py
```

Open `http://<vm-ip>:8889/watchpass` in a browser to watch the WebRTC stream.

### Chained processing demo

`camera_bridge → grayscale → flip → ffmpeg`, all in one container, every hop
zero-copy:

```bash
ros2 launch watchpass_bringup chained.launch.py \
  image_topic:=/my_camera/image_raw flip_code:=1
```

### Pipeline only (bring your own camera)

```bash
ros2 launch watchpass_bringup stream.launch.py image_topic:=/my_camera/image_raw
```

### GPU (VA-API) encoding — AMD / Intel

Offload H.264 encoding to the GPU (`h264_vaapi`) instead of the CPU:

```bash
ros2 launch watchpass_bringup stream.launch.py \
  encoder:=vaapi vaapi_device:=/dev/dri/renderD128
```

Needs the VA-API stack (`mesa-va-drivers`, `libva2`, `vainfo`) and membership of
the `render` group; check with `vainfo`. The fully zero-copy DMABUF-to-GPU path
is described in [docs/design_generic_transport.md](docs/design_generic_transport.md).

### Change the output

`output_args` overrides the encoder preset entirely — e.g. raw RTP:

```bash
ros2 run watchpass_streamer ffmpeg_streamer --ros-args \
  -p 'output_args:=[-c:v, libx264, -tune, zerolatency, -f, rtp, rtp://127.0.0.1:5004]'
```

…or test the encoder stage alone with the reference script:

```bash
mkfifo /tmp/watchpass.raw
src/watchpass_streamer/scripts/stream_ffmpeg.sh --input /tmp/watchpass.raw \
  --size 640x480 --fps 30 --target rtsp://127.0.0.1:8554/watchpass
```

## Writing your own processing node

Subclass `FrameProcessorNode` and implement one method. You get a zero-copy input
`FrameView`, a loaned output `WatchFrame` (header already copied), and OpenCV via
`as_mat()` — no boilerplate, no copies:

```cpp
class InvertNode : public watchpass::FrameProcessorNode {
public:
  explicit InvertNode(const rclcpp::NodeOptions & o)
  : FrameProcessorNode("invert", o) {}
protected:
  bool process(const watchpass::FrameView & in,
               watchpass_msgs::msg::WatchFrame & out) override {
    cv::Mat src = in.as_mat();            // aliases the loaned input buffer
    if (src.empty()) return false;
    copy_geometry(in, out);
    out.data_size = in.size();
    cv::Mat dst(out.height, out.width, watchpass::FrameFormat::cv_type(in.encoding()),
                out.data.data(), out.step);   // aliases the loaned output buffer
    cv::bitwise_not(src, dst);            // writes straight into shared memory
    return true;
  }
};
RCLCPP_COMPONENTS_REGISTER_NODE(InvertNode)
```

Drop it into a container between any two stages. Terminal sinks (recorders,
detectors) subclass `FrameConsumerNode` and implement `consume(const FrameView&)`
instead — that's exactly what `ffmpeg_streamer` is.

### Changing frame capacity

The payload size is a build-time knob, not a hardcoded literal:

```bash
colcon build --packages-select watchpass_msgs \
  --cmake-args -DWATCHPASS_FRAME_CAPACITY=6220800   # 1920x1080x3
```

Both `MAX_DATA` and the array bound are generated from it, so they can't drift.
Size it to your largest frame — the capacity is preallocated per shared buffer.

## Verify the zero-copy path

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=\
$(ros2 pkg prefix watchpass_bringup)/share/watchpass_bringup/config/fastdds_shm.xml
ros2 run watchpass_streamer camera_bridge   # look for "loaned messages ENABLED"
```

If it instead prints `unavailable (middleware copies)`, the SHM profile isn't
loaded or the message type isn't plain — check `FASTRTPS_DEFAULT_PROFILES_FILE`
and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.

## Configuration reference

| Where | Key | Default | Meaning |
| --- | --- | --- | --- |
| `camera_bridge` | `input_topic` / `output_topic` | `image_raw` / `watch_frame` | Source image → published `WatchFrame`. |
| `FrameProcessorNode` | `input_topic` / `output_topic` | `watch_frame` / `watch_frame_out` | In/out topics for any processor (grayscale, flip, …). |
| `flip` | `flip_code` | `1` | `0`=vertical, `1`=horizontal, `-1`=both. |
| `ffmpeg_streamer` | `input_topic` | `watch_frame` | Source `WatchFrame`. |
| `ffmpeg_streamer` | `fps` | `30.0` | Rate advertised to ffmpeg. |
| `ffmpeg_streamer` | `encoder` | `x264` | `x264` (software) or `vaapi` (AMD/Intel GPU). |
| `ffmpeg_streamer` | `rtsp_url` | `rtsp://127.0.0.1:8554/watchpass` | Destination for the preset encoders. |
| `ffmpeg_streamer` | `vaapi_device` | `/dev/dri/renderD128` | VA-API render node for `encoder:=vaapi`. |
| `ffmpeg_streamer` | `output_args` | *(empty)* | Non-empty overrides the preset: argv after ffmpeg's input spec. |
| build (CMake) | `WATCHPASS_FRAME_CAPACITY` | `2764800` | `WatchFrame` payload capacity (1280×720×3). |

## License

Apache-2.0. See [LICENSE](LICENSE). Contributions welcome — see
[CONTRIBUTING.md](CONTRIBUTING.md).
