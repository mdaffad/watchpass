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
#include "watchpass_framework/frame_view.hpp"

#include <string>

#include "opencv2/core.hpp"

namespace watchpass
{

using WatchFrame = watchpass_msgs::msg::WatchFrame;

int FrameFormat::channels(uint8_t encoding)
{
  switch (encoding) {
    case WatchFrame::ENCODING_RGB8:
    case WatchFrame::ENCODING_BGR8:
      return 3;
    case WatchFrame::ENCODING_MONO8:
      return 1;
    default:
      return 0;
  }
}

int FrameFormat::cv_type(uint8_t encoding)
{
  switch (encoding) {
    case WatchFrame::ENCODING_RGB8:
    case WatchFrame::ENCODING_BGR8:
      return CV_8UC3;
    case WatchFrame::ENCODING_MONO8:
      return CV_8UC1;
    default:
      return -1;
  }
}

const char * FrameFormat::ffmpeg_pix_fmt(uint8_t encoding)
{
  switch (encoding) {
    case WatchFrame::ENCODING_RGB8: return "rgb24";
    case WatchFrame::ENCODING_BGR8: return "bgr24";
    case WatchFrame::ENCODING_MONO8: return "gray";
    default: return nullptr;
  }
}

const char * FrameFormat::name(uint8_t encoding)
{
  switch (encoding) {
    case WatchFrame::ENCODING_RGB8: return "rgb8";
    case WatchFrame::ENCODING_BGR8: return "bgr8";
    case WatchFrame::ENCODING_MONO8: return "mono8";
    default: return "unknown";
  }
}

bool FrameFormat::from_ros(const std::string & ros_encoding, uint8_t & out)
{
  if (ros_encoding == "rgb8") {
    out = WatchFrame::ENCODING_RGB8;
  } else if (ros_encoding == "bgr8") {
    out = WatchFrame::ENCODING_BGR8;
  } else if (ros_encoding == "mono8") {
    out = WatchFrame::ENCODING_MONO8;
  } else {
    return false;
  }
  return true;
}

bool FrameView::valid() const
{
  const int ch = channels();
  if (ch == 0) {
    return false;
  }
  if (msg_.width == 0 || msg_.height == 0) {
    return false;
  }
  if (msg_.step < msg_.width * static_cast<uint32_t>(ch)) {
    return false;
  }
  const size_t needed = static_cast<size_t>(msg_.step) * msg_.height;
  if (msg_.data_size < needed || msg_.data_size > msg_.data.size()) {
    return false;
  }
  return true;
}

cv::Mat FrameView::as_mat() const
{
  const int type = FrameFormat::cv_type(msg_.encoding);
  if (type < 0 || !valid()) {
    return cv::Mat();
  }
  // Alias the message buffer: no allocation, no copy. Caller treats it as const.
  return cv::Mat(
    static_cast<int>(msg_.height), static_cast<int>(msg_.width), type,
    const_cast<uint8_t *>(msg_.data.data()), static_cast<size_t>(msg_.step));
}

}  // namespace watchpass
