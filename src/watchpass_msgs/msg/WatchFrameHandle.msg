# WatchFrameHandle references pixels that live OUTSIDE the message, in an
# externally-managed buffer. Use it when inlining the payload is undesirable or
# impossible:
#   * frames larger than any fixed WatchFrame capacity (4K and up),
#   * GPU surfaces that should never touch host memory (AMD VAAPI / DMABUF).
#
# The handle itself is tiny metadata (it is NOT the zero-copy hot path). The
# zero-copy property lives in the referenced buffer: a host shared-memory ring
# pool, or a DMABUF exported by the GPU. Producer and consumer agree on the pool
# out-of-band (same host); the consumer maps the pool / imports the DMABUF fd and
# reads the pixels without a copy.
#
# This decouples frame size from the message entirely, at the cost of managing
# the pool lifecycle yourself. See docs/design_generic_transport.md.

# --- Transport kind --------------------------------------------------
uint8 KIND_SHM_POOL=0    # pixels in a named POSIX shared-memory ring buffer
uint8 KIND_DMABUF=1      # pixels in a GPU DMABUF (fd passed same-host)

# --- Header ----------------------------------------------------------
builtin_interfaces/Time stamp
uint32 seq

# --- Geometry --------------------------------------------------------
uint32 width
uint32 height
uint32 step
uint8  encoding          # same ENCODING_* meaning as WatchFrame

# --- Reference -------------------------------------------------------
uint8  kind              # one of KIND_*
string pool_name         # shm object name / pool identifier
uint32 buffer_index      # slot within the pool ring
uint64 offset            # byte offset of the frame within the segment
uint32 data_size         # valid bytes
int32  dmabuf_fd         # GPU buffer fd when kind == KIND_DMABUF, else -1
