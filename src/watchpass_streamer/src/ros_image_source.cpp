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
#include "watchpass_streamer/ros_image_source.hpp"

#include <string>
#include <utility>

#include "watchpass_framework/frame_view.hpp"

namespace watchpass_streamer
{

RosImageSource::RosImageSource(rclcpp::Node & node, const std::string & topic)
: node_(node), topic_(topic) {}

void RosImageSource::start(Callback cb)
{
  cb_ = std::move(cb);
  sub_ = node_.create_subscription<sensor_msgs::msg::Image>(
    topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {on_image(msg);});
  RCLCPP_INFO(node_.get_logger(), "source=topic: subscribing '%s'", topic_.c_str());
}

void RosImageSource::stop()
{
  sub_.reset();
}

void RosImageSource::on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  uint8_t encoding = 0;
  if (!watchpass::FrameFormat::from_ros(msg->encoding, encoding)) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "unsupported encoding '%s' (expected rgb8/bgr8/mono8), dropping frame",
      msg->encoding.c_str());
    return;
  }

  watchpass::RawFrame f;
  f.width = msg->width;
  f.height = msg->height;
  f.step = msg->step;
  f.encoding = encoding;
  f.data = msg->data.data();
  f.size = msg->data.size();
  f.stamp = msg->header.stamp;
  f.has_stamp = true;
  cb_(f);
}

}  // namespace watchpass_streamer
