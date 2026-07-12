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
#ifndef WATCHPASS_FRAMEWORK__FRAME_SOURCE_HPP_
#define WATCHPASS_FRAMEWORK__FRAME_SOURCE_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>

#include "builtin_interfaces/msg/time.hpp"

namespace watchpass
{

/// A raw frame handed from a FrameSource to the bridge.
///
/// Non-owning: `data` points at memory owned by the source (a ROS message's
/// buffer, a V4L2 mmap slot, ...) and is only guaranteed valid for the duration
/// of the callback. The bridge copies it exactly once into a loaned WatchFrame,
/// which is the single unavoidable copy on the ingest path.
struct RawFrame
{
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t step = 0;         ///< bytes per row; 0 means width * channels
  uint8_t encoding = 0;      ///< a WatchFrame::ENCODING_* value
  const uint8_t * data = nullptr;
  size_t size = 0;           ///< valid bytes at `data`
  builtin_interfaces::msg::Time stamp;
  bool has_stamp = false;    ///< false => bridge stamps the frame with now()
};

/// Abstract capture source feeding the camera bridge.
///
/// Push model: the source invokes the callback once per captured frame, from
/// whatever thread or executor it uses (a subscription callback for a ROS topic,
/// a dedicated capture thread for a device). Concrete sources live in the
/// packages that own their transport (e.g. RosImageSource / V4l2Source in
/// watchpass_streamer), so this contract stays dependency-light.
class FrameSource
{
public:
  using Callback = std::function<void (const RawFrame &)>;

  virtual ~FrameSource() = default;

  /// Begin producing frames, delivered to `cb`. Must be called once.
  virtual void start(Callback cb) = 0;

  /// Stop producing frames and release resources. Safe to call more than once.
  virtual void stop() = 0;
};

}  // namespace watchpass

#endif  // WATCHPASS_FRAMEWORK__FRAME_SOURCE_HPP_
