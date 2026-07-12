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
#ifndef WATCHPASS_STREAMER__ROS_IMAGE_SOURCE_HPP_
#define WATCHPASS_STREAMER__ROS_IMAGE_SOURCE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "watchpass_framework/frame_source.hpp"

namespace watchpass_streamer
{

/// FrameSource that subscribes to a sensor_msgs/Image topic (the original
/// behavior). Each incoming Image is mapped to a RawFrame that aliases the
/// message buffer and delivered via the callback on the executor thread.
class RosImageSource : public watchpass::FrameSource
{
public:
  RosImageSource(rclcpp::Node & node, const std::string & topic);

  void start(Callback cb) override;
  void stop() override;

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Node & node_;
  std::string topic_;
  Callback cb_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace watchpass_streamer

#endif  // WATCHPASS_STREAMER__ROS_IMAGE_SOURCE_HPP_
