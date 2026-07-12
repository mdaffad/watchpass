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
//
// ffmpeg_streamer: a FrameConsumerNode that pipes raw WatchFrame pixels into
// ffmpeg for encoding + transport to a WebRTC/RTSP/RTP server.
//
// As a consumer it inherits the zero-copy subscription from the framework; its
// only job is to feed the encoder over a pipe. The encoder is selectable:
//   * encoder=x264  -> software H.264 (libx264), portable default.
//   * encoder=vaapi -> AMD/Intel GPU H.264 (h264_vaapi) via VA-API. Frames are
//                      uploaded to the GPU (format=nv12,hwupload) and encoded
//                      there, offloading the CPU. For a full DMABUF-in path with
//                      no upload, see WatchFrameHandle + docs.
// Setting `output_args` overrides the preset entirely for full control.

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "watchpass_framework/frame_consumer_node.hpp"
#include "watchpass_framework/frame_view.hpp"
#include "watchpass_streamer/ffmpeg_pipe.hpp"

namespace watchpass_streamer
{

class FfmpegStreamerNode : public watchpass::FrameConsumerNode
{
public:
  explicit FfmpegStreamerNode(const rclcpp::NodeOptions & options)
  : watchpass::FrameConsumerNode("ffmpeg_streamer", options)
  {
    ffmpeg_path_ = declare_parameter<std::string>("ffmpeg_path", "ffmpeg");
    fps_ = declare_parameter<double>("fps", 30.0);
    encoder_ = declare_parameter<std::string>("encoder", "x264");    // x264 | vaapi
    rtsp_url_ = declare_parameter<std::string>("rtsp_url", "rtsp://127.0.0.1:8554/watchpass");
    vaapi_device_ = declare_parameter<std::string>("vaapi_device", "/dev/dri/renderD128");
    // Non-empty output_args fully overrides the encoder preset.
    output_args_ = declare_parameter<std::vector<std::string>>(
      "output_args", std::vector<std::string>{});
  }

  ~FfmpegStreamerNode() override
  {
    pipe_.stop();
  }

protected:
  void consume(const watchpass::FrameView & frame) override
  {
    const char * pix = watchpass::FrameFormat::ffmpeg_pix_fmt(frame.encoding());
    if (pix == nullptr) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "unknown encoding %u, dropping", frame.encoding());
      return;
    }

    if (!pipe_.running() || geometry_changed(frame, pix)) {
      if (pipe_.running()) {
        RCLCPP_INFO(get_logger(), "geometry changed, restarting ffmpeg");
        pipe_.stop();
      }
      FfmpegInput in;
      in.width = frame.width();
      in.height = frame.height();
      in.fps = fps_;
      in.pixel_format = pix;
      if (!pipe_.start(ffmpeg_path_, in, build_output_args())) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "failed to start ffmpeg ('%s')",
          ffmpeg_path_.c_str());
        return;
      }
      RCLCPP_INFO(
        get_logger(), "ffmpeg started (%s) for %ux%u %s @ %.1f fps -> %s",
        encoder_.c_str(), in.width, in.height, in.pixel_format.c_str(), in.fps,
        rtsp_url_.c_str());
    }

    if (!pipe_.write_frame(frame.data(), frame.size())) {
      RCLCPP_WARN(get_logger(), "ffmpeg pipe broke, will relaunch on next frame");
      pipe_.stop();
    }
  }

private:
  bool geometry_changed(const watchpass::FrameView & f, const char * pix) const
  {
    const auto & in = pipe_.input();
    return in.width != f.width() || in.height != f.height() || in.pixel_format != pix;
  }

  std::vector<std::string> build_output_args() const
  {
    if (!output_args_.empty()) {
      return output_args_;  // full manual override
    }
    if (encoder_ == "vaapi") {
      // AMD/Intel GPU H.264. Upload the CPU frame to a VA-API surface, encode there.
      return {
        "-vaapi_device", vaapi_device_,
        "-vf", "format=nv12,hwupload",
        "-c:v", "h264_vaapi",
        "-f", "rtsp", "-rtsp_transport", "tcp", rtsp_url_};
    }
    // Software H.264 (default).
    return {
      "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
      "-pix_fmt", "yuv420p",
      "-f", "rtsp", "-rtsp_transport", "tcp", rtsp_url_};
  }

  std::string ffmpeg_path_;
  double fps_ = 30.0;
  std::string encoder_;
  std::string rtsp_url_;
  std::string vaapi_device_;
  std::vector<std::string> output_args_;
  FfmpegPipe pipe_;
};

}  // namespace watchpass_streamer

RCLCPP_COMPONENTS_REGISTER_NODE(watchpass_streamer::FfmpegStreamerNode)
