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
// camera_bridge: ingest camera frames into a fixed-size, zero-copy WatchFrame.
//
// The frame source is pluggable (parameter `source`):
//   source=topic -> RosImageSource: subscribe to a sensor_msgs/Image topic
//                   (e.g. from ros_gz_bridge / a camera driver node).
//   source=v4l2  -> V4l2Source: capture straight from /dev/videoN, skipping the
//                   driver node and its serialize/publish/deserialize hop.
//
// Whatever the source, the bridge borrows a loaned WatchFrame from the
// middleware's shared-memory pool, copies the pixels in exactly once, and
// publishes without serialization. Downstream nodes on the same host receive the
// frame with no further copy.

#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "watchpass_framework/frame_format.hpp"
#include "watchpass_framework/frame_source.hpp"
#include "watchpass_msgs/msg/watch_frame.hpp"
#include "watchpass_streamer/ros_image_source.hpp"
#include "watchpass_streamer/v4l2_source.hpp"

namespace watchpass_streamer
{

class CameraBridgeNode : public rclcpp::Node
{
public:
  explicit CameraBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("camera_bridge", options)
  {
    const std::string source = declare_parameter<std::string>("source", "topic");
    output_topic_ = declare_parameter<std::string>("output_topic", "watch_frame");

    pub_ = create_publisher<watchpass_msgs::msg::WatchFrame>(
      output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(), "camera_bridge: source=%s -> %s (WatchFrame), loaned messages %s",
      source.c_str(), output_topic_.c_str(),
      pub_->can_loan_messages() ? "ENABLED (zero-copy)" : "unavailable (middleware copies)");

    source_ = make_source(source);
    if (source_) {
      source_->start([this](const watchpass::RawFrame & f) {publish_frame(f);});
    }
  }

  ~CameraBridgeNode() override
  {
    if (source_) {
      source_->stop();
    }
  }

private:
  std::unique_ptr<watchpass::FrameSource> make_source(const std::string & source)
  {
    if (source == "topic") {
      const auto topic = declare_parameter<std::string>("input_topic", "image_raw");
      return std::make_unique<RosImageSource>(*this, topic);
    }
    if (source == "v4l2") {
      V4l2Source::Config cfg;
      cfg.device = declare_parameter<std::string>("device", "/dev/video0");
      cfg.width = static_cast<uint32_t>(declare_parameter<int>("width", 640));
      cfg.height = static_cast<uint32_t>(declare_parameter<int>("height", 480));
      cfg.fps = declare_parameter<double>("fps", 30.0);
      const auto enc = declare_parameter<std::string>("pixel_format", "rgb8");
      if (!watchpass::FrameFormat::from_ros(enc, cfg.encoding)) {
        RCLCPP_ERROR(
          get_logger(), "invalid pixel_format '%s' (expected rgb8/bgr8/mono8)", enc.c_str());
        return nullptr;
      }
      return std::make_unique<V4l2Source>(*this, cfg);
    }
    RCLCPP_ERROR(get_logger(), "unknown source '%s' (expected topic|v4l2)", source.c_str());
    return nullptr;
  }

  // Single publish path shared by every source: validate, borrow a loaned
  // buffer, copy the pixels in once, publish zero-copy.
  void publish_frame(const watchpass::RawFrame & f)
  {
    const int channels = watchpass::FrameFormat::channels(f.encoding);
    if (channels == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "frame with unknown encoding %u, dropping", f.encoding);
      return;
    }
    const uint32_t step = f.step ? f.step : f.width * static_cast<uint32_t>(channels);
    const size_t needed = static_cast<size_t>(step) * f.height;
    constexpr size_t kCapacity = watchpass_msgs::msg::WatchFrame::MAX_DATA;
    if (needed > kCapacity) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "frame %ux%u (%zu bytes) exceeds WatchFrame capacity %zu; drop or raise "
        "WATCHPASS_FRAME_CAPACITY", f.width, f.height, needed, kCapacity);
      return;
    }
    if (f.size < needed) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "short source buffer; dropping frame");
      return;
    }

    auto loaned = pub_->borrow_loaned_message();
    auto & frame = loaned.get();

    if (f.has_stamp) {
      frame.stamp = f.stamp;
    } else {
      frame.stamp = now();
    }
    frame.seq = seq_++;
    frame.width = f.width;
    frame.height = f.height;
    frame.step = step;
    frame.encoding = f.encoding;
    frame.data_size = static_cast<uint32_t>(needed);

    // The single unavoidable copy: source pixels -> loaned shared buffer.
    std::memcpy(frame.data.data(), f.data, needed);

    pub_->publish(std::move(loaned));
  }

  std::string output_topic_;
  uint32_t seq_ = 0;
  std::unique_ptr<watchpass::FrameSource> source_;
  rclcpp::Publisher<watchpass_msgs::msg::WatchFrame>::SharedPtr pub_;
};

}  // namespace watchpass_streamer

RCLCPP_COMPONENTS_REGISTER_NODE(watchpass_streamer::CameraBridgeNode)
