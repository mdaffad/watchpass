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
#ifndef WATCHPASS_FRAMEWORK__FRAME_FORMAT_HPP_
#define WATCHPASS_FRAMEWORK__FRAME_FORMAT_HPP_

#include <cstdint>
#include <string>

namespace watchpass
{

/// Static helpers that translate WatchFrame encodings to the vocabularies used
/// by OpenCV and ffmpeg. The single source of truth for encodings, so every
/// node (bridge, processors, streamer) agrees. Declared without an OpenCV
/// include so lightweight consumers (e.g. camera_bridge) need not depend on it;
/// cv_type() simply returns the integer OpenCV type code.
struct FrameFormat
{
  /// Bytes per pixel for an encoding, or 0 if unknown.
  static int channels(uint8_t encoding);
  /// OpenCV matrix type (e.g. CV_8UC3 == 16), or -1 if unknown.
  static int cv_type(uint8_t encoding);
  /// ffmpeg raw pixel format string (e.g. "rgb24"), or nullptr if unknown.
  static const char * ffmpeg_pix_fmt(uint8_t encoding);
  /// Human-readable name, e.g. "rgb8".
  static const char * name(uint8_t encoding);
  /// Map a ROS sensor_msgs/Image encoding string to a WatchFrame encoding.
  /// Returns false if unsupported.
  static bool from_ros(const std::string & ros_encoding, uint8_t & out);
};

}  // namespace watchpass

#endif  // WATCHPASS_FRAMEWORK__FRAME_FORMAT_HPP_
