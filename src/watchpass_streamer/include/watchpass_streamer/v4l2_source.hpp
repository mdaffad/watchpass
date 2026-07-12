// Copyright 2026 Muhammad Daffa Dinaya
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef WATCHPASS_STREAMER__V4L2_SOURCE_HPP_
#define WATCHPASS_STREAMER__V4L2_SOURCE_HPP_

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "watchpass_framework/frame_source.hpp"

namespace watchpass_streamer
{

/// FrameSource that captures directly from a V4L2 device (/dev/videoN) using
/// memory-mapped streaming I/O, bypassing any camera-driver node and its
/// serialize/publish/deserialize round-trip. A dedicated thread dequeues each
/// filled kernel buffer, hands it to the callback as a RawFrame aliasing the
/// mmap slot, then requeues it.
///
/// Only directly-mappable pixel formats are supported (RGB24/BGR24/GREY, which
/// map 1:1 to the WatchFrame encodings). Cameras that only emit YUYV or MJPEG
/// need a conversion step (libv4l or a FrameProcessorNode) -- see the design
/// notes; start() reports a clear error if the negotiated format is unsupported.
class V4l2Source : public watchpass::FrameSource
{
public:
  struct Config
  {
    std::string device = "/dev/video0";
    uint32_t width = 640;
    uint32_t height = 480;
    uint8_t encoding = 0;      ///< requested WatchFrame::ENCODING_* (RGB8 default)
    double fps = 30.0;
    uint32_t buffer_count = 4;
  };

  V4l2Source(rclcpp::Node & node, const Config & config);
  ~V4l2Source() override;

  void start(Callback cb) override;
  void stop() override;

private:
  struct MappedBuffer
  {
    void * start = nullptr;
    size_t length = 0;
  };

  bool open_device();
  bool negotiate_format();     ///< S_FMT + read back; sets negotiated_*
  bool set_frame_rate();       ///< best-effort S_PARM
  bool request_buffers();      ///< REQBUFS + QUERYBUF + mmap + QBUF
  bool start_streaming();
  void capture_loop();
  void teardown();

  rclcpp::Node & node_;
  Config config_;
  Callback cb_;

  int fd_ = -1;
  std::vector<MappedBuffer> buffers_;
  std::atomic<bool> running_{false};
  std::thread thread_;

  // Format actually negotiated with the driver.
  uint32_t negotiated_width_ = 0;
  uint32_t negotiated_height_ = 0;
  uint32_t negotiated_step_ = 0;
  uint8_t negotiated_encoding_ = 0;
  uint32_t seq_ = 0;
};

}  // namespace watchpass_streamer

#endif  // WATCHPASS_STREAMER__V4L2_SOURCE_HPP_
