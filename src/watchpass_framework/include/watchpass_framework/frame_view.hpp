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
#ifndef WATCHPASS_FRAMEWORK__FRAME_VIEW_HPP_
#define WATCHPASS_FRAMEWORK__FRAME_VIEW_HPP_

#include <cstddef>
#include <cstdint>

#include "opencv2/core/mat.hpp"
#include "watchpass_framework/frame_format.hpp"
#include "watchpass_msgs/msg/watch_frame.hpp"

namespace watchpass
{

/// Non-owning, read-only view over a WatchFrame.
///
/// FrameView never copies pixels. `as_mat()` returns a cv::Mat that aliases the
/// message's buffer, so a processing node can run OpenCV on frames that arrived
/// zero-copy over shared memory without ever duplicating them.
class FrameView
{
public:
  explicit FrameView(const watchpass_msgs::msg::WatchFrame & msg)
  : msg_(msg) {}

  uint32_t width() const {return msg_.width;}
  uint32_t height() const {return msg_.height;}
  uint32_t step() const {return msg_.step;}
  uint8_t encoding() const {return msg_.encoding;}
  int channels() const {return FrameFormat::channels(msg_.encoding);}
  uint32_t seq() const {return msg_.seq;}
  const builtin_interfaces::msg::Time & stamp() const {return msg_.stamp;}

  const uint8_t * data() const {return msg_.data.data();}
  /// Number of valid bytes (not the fixed capacity).
  size_t size() const {return msg_.data_size;}

  /// True when geometry, encoding and data_size are mutually consistent and fit
  /// within the message capacity.
  bool valid() const;

  /// cv::Mat aliasing the frame buffer (no copy). Empty Mat if the encoding is
  /// unsupported or the frame is invalid. The returned Mat is logically const;
  /// treat it as read-only.
  cv::Mat as_mat() const;

  const watchpass_msgs::msg::WatchFrame & message() const {return msg_;}

private:
  const watchpass_msgs::msg::WatchFrame & msg_;
};

}  // namespace watchpass

#endif  // WATCHPASS_FRAMEWORK__FRAME_VIEW_HPP_
