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
#include "watchpass_framework/frame_consumer_node.hpp"

#include <string>

namespace watchpass
{

FrameConsumerNode::FrameConsumerNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  input_topic_ = declare_parameter<std::string>("input_topic", "watch_frame");
  sub_ = create_subscription<watchpass_msgs::msg::WatchFrame>(
    input_topic_, rclcpp::SensorDataQoS(),
    [this](const watchpass_msgs::msg::WatchFrame::ConstSharedPtr msg) {on_frame(msg);});
  RCLCPP_INFO(get_logger(), "%s: consuming '%s'", node_name.c_str(), input_topic_.c_str());
}

void FrameConsumerNode::on_frame(const watchpass_msgs::msg::WatchFrame::ConstSharedPtr msg)
{
  FrameView view(*msg);
  if (!view.valid()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "invalid frame, dropping");
    return;
  }
  consume(view);
}

}  // namespace watchpass
