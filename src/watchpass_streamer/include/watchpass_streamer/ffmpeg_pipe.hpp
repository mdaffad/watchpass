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
#ifndef WATCHPASS_STREAMER__FFMPEG_PIPE_HPP_
#define WATCHPASS_STREAMER__FFMPEG_PIPE_HPP_

#include <sys/types.h>

#include <cstdint>
#include <string>
#include <vector>

namespace watchpass_streamer
{

/// Parameters describing the raw video that will be fed into ffmpeg's stdin.
struct FfmpegInput
{
  uint32_t width = 0;
  uint32_t height = 0;
  double fps = 30.0;
  /// ffmpeg raw pixel format, e.g. "rgb24", "bgr24", "gray".
  std::string pixel_format = "rgb24";
};

/// Manages an ffmpeg child process fed through an anonymous pipe.
///
/// The node keeps the write end of the pipe; ffmpeg reads raw frames from the
/// read end wired to its stdin (`-i pipe:0`). This is the interprocess transfer
/// stage: ROS delivers frames to the streamer node zero-copy over shared memory,
/// and the streamer hands the raw bytes to the encoder over a kernel pipe.
class FfmpegPipe
{
public:
  FfmpegPipe() = default;
  ~FfmpegPipe();

  FfmpegPipe(const FfmpegPipe &) = delete;
  FfmpegPipe & operator=(const FfmpegPipe &) = delete;

  /// Launch ffmpeg.
  /// \param ffmpeg_path   executable to exec (usually "ffmpeg").
  /// \param input         raw video description used to build the input args.
  /// \param output_args   everything after the input spec (codec + muxer + URL),
  ///                       already tokenized into argv entries.
  /// \returns true on success.
  bool start(
    const std::string & ffmpeg_path,
    const FfmpegInput & input,
    const std::vector<std::string> & output_args);

  /// Write one raw frame to ffmpeg. Returns false if the pipe broke (ffmpeg died).
  bool write_frame(const uint8_t * data, size_t size);

  /// Close the pipe and reap the child. Safe to call multiple times.
  void stop();

  bool running() const {return pid_ > 0;}
  const FfmpegInput & input() const {return input_;}

private:
  pid_t pid_ = -1;
  int write_fd_ = -1;
  FfmpegInput input_;
};

}  // namespace watchpass_streamer

#endif  // WATCHPASS_STREAMER__FFMPEG_PIPE_HPP_
