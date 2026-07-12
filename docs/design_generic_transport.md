# Generic frame transport & the AMD VAAPI path

The default pipeline carries pixels *inside* a fixed-size `WatchFrame`. That is
the right choice for a huge range of cameras because it keeps the message
**plain**, which is the precondition for true zero-copy loaned messages. But it
has two known limits, and this document explains how the design scales past
them.

## The core constraint

| Property | Requirement |
| --- | --- |
| Zero-copy **loan** (`borrow_loaned_message`, no copy at all) | message is **plain** — fixed-size only |
| Shared-memory **data-sharing** (copy into a shared segment, no network) | message is **bounded** |
| Serialize + network | any message |

`WatchFrame` is plain, so it gets the top row. Making the payload a variable
sequence (`uint8[<=N]`) would demote it to "bounded" and silently turn every
publish into a copy. So the size must stay fixed — the knob below manages that.

## Scaling the size: `WATCHPASS_FRAME_CAPACITY`

The payload capacity is a build-time value, not a literal:

```bash
# 1080p RGB
colcon build --packages-select watchpass_msgs \
  --cmake-args -DWATCHPASS_FRAME_CAPACITY=6220800
```

Both the `MAX_DATA` constant and the `uint8[N]` bound are generated from this one
value, so they cannot drift. Trade-off: the whole capacity is preallocated per
buffer in the shared-memory pool, so size it to your **largest** frame, not
larger.

## Scaling past inlining: `WatchFrameHandle`

When frames are too big to inline sensibly (4K) or must never touch host memory
(GPU surfaces), switch the *currency* from "pixels in the message" to "a
reference to pixels":

```
WatchFrameHandle:  kind, pool_name, buffer_index, offset, data_size, dmabuf_fd, geometry...
```

The handle is tiny metadata. The zero-copy property moves into the referenced
buffer:

- `KIND_SHM_POOL` — a producer-owned POSIX shared-memory **ring** of N slots.
  The handle names the pool and the slot; the consumer `mmap`s the pool once and
  reads the slot directly. Decouples frame size from any message bound, portable,
  host-only.
- `KIND_DMABUF` — the pixels live in a GPU buffer exported as a **DMABUF** fd.
  The fd is passed to same-host consumers, which import it into their own GPU
  context. No host copy, no upload.

This is a deliberate extension point, not yet a runtime in this repo: the message
type ships, the pool/dmabuf lifecycle does not. Implement it when a camera or GPU
source actually produces these buffers.

## AMD GPU encoding with VAAPI

Two levels, independent of each other:

### 1. Offload encoding (works today, over the existing pipe)

`ffmpeg_streamer` has a `vaapi` encoder preset. Raw frames still arrive over the
pipe (CPU), get uploaded to a VA-API surface once, and H.264 encoding runs on the
AMD GPU:

```bash
ros2 launch watchpass_bringup stream.launch.py \
  encoder:=vaapi vaapi_device:=/dev/dri/renderD128
```

which builds:

```
ffmpeg ... -i pipe:0 -vaapi_device /dev/dri/renderD128 \
       -vf format=nv12,hwupload -c:v h264_vaapi -f rtsp <url>
```

Requires the AMD stack in the VM: `mesa-va-drivers`, `libva2`, `vainfo`, and the
user in the `render` group. Verify with `vainfo` (expect `VAProfileH264*`).

### 2. Fully zero-copy to the GPU (the DMABUF path)

The end goal: a capture/render source exports a DMABUF, publishes a
`WatchFrameHandle(KIND_DMABUF)`, and `ffmpeg_streamer` imports that fd straight
into a VA-API surface (`-hwaccel vaapi -hwaccel_output_format vaapi` with a
DMABUF import) — no pipe, no upload, no host copy on the entire path from sensor
to encoder. This is where `WatchFrameHandle` pays off and is the natural next
implementation step.

## Where each piece lives

| Concern | Mechanism | Status |
| --- | --- | --- |
| Small/medium frames, host | plain `WatchFrame` + loaned messages | implemented, verified |
| Configurable max size | `WATCHPASS_FRAME_CAPACITY` | implemented |
| Reusable processing nodes | `FrameProcessorNode` / `FrameConsumerNode` | implemented |
| GPU-offloaded encode | `encoder:=vaapi` (`h264_vaapi`) | implemented (needs AMD stack) |
| 4K / no-inline / GPU surfaces | `WatchFrameHandle` (shm pool / DMABUF) | message + design; runtime TODO |
