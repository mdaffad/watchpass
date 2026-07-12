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
// A real camera/image driver would fill the loaned WatchFrame buffer directly
// from its DMA capture buffer. In simulation the frames arrive as
// sensor_msgs/Image (e.g. from ros_gz_bridge / Gazebo), so this node bridges
// them: it borrows a loaned message from the middleware's shared-memory pool,
// copies the pixels in once, and publishes without serialization. Downstream
// subscribers on the same host then receive the frame with no further copy.

#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "watchpass_framework/frame_view.hpp"
#include "watchpass_msgs/msg/watch_frame.hpp"

namespace watchpass_streamer
{

class CameraBridgeNode : public rclcpp::Node
{
public:
  explicit CameraBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("camera_bridge", options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "image_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "watch_frame");

    auto qos = rclcpp::SensorDataQoS();
    pub_ = create_publisher<watchpass_msgs::msg::WatchFrame>(output_topic_, qos);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_, qos,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {on_image(*msg);});

    RCLCPP_INFO(
      get_logger(),
      "camera_bridge: %s (sensor_msgs/Image) -> %s (WatchFrame), loaned messages %s",
      input_topic_.c_str(), output_topic_.c_str(),
      pub_->can_loan_messages() ? "ENABLED (zero-copy)" : "unavailable (middleware copies)");
  }

private:
  void on_image(const sensor_msgs::msg::Image & img)
  {
    uint8_t encoding = 0;
    if (!watchpass::FrameFormat::from_ros(img.encoding, encoding)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "unsupported encoding '%s' (expected rgb8/bgr8/mono8), dropping frame",
        img.encoding.c_str());
      return;
    }
    const int channels = watchpass::FrameFormat::channels(encoding);

    const uint32_t step = img.step ? img.step : img.width * static_cast<uint32_t>(channels);
    const size_t needed = static_cast<size_t>(step) * img.height;
    constexpr size_t kCapacity = watchpass_msgs::msg::WatchFrame::MAX_DATA;
    if (needed > kCapacity) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "frame %ux%u (%zu bytes) exceeds WatchFrame capacity %zu; drop or raise "
        "WATCHPASS_FRAME_CAPACITY", img.width, img.height, needed, kCapacity);
      return;
    }
    if (img.data.size() < needed) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "short image buffer; dropping frame");
      return;
    }

    // Borrow a buffer from the middleware. When shared-memory transport is
    // active this is a slot in the shared pool, so the publish below performs
    // no serialization and no cross-process copy.
    auto loaned = pub_->borrow_loaned_message();
    auto & frame = loaned.get();

    frame.stamp = img.header.stamp;
    frame.seq = seq_++;
    frame.width = img.width;
    frame.height = img.height;
    frame.step = step;
    frame.encoding = encoding;
    frame.data_size = static_cast<uint32_t>(needed);

    // The single unavoidable copy: source pixels -> loaned shared buffer.
    std::memcpy(frame.data.data(), img.data.data(), needed);

    pub_->publish(std::move(loaned));
  }

  std::string input_topic_;
  std::string output_topic_;
  uint32_t seq_ = 0;
  rclcpp::Publisher<watchpass_msgs::msg::WatchFrame>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace watchpass_streamer

RCLCPP_COMPONENTS_REGISTER_NODE(watchpass_streamer::CameraBridgeNode)
