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
#include "watchpass_framework/frame_processor_node.hpp"

#include <string>
#include <utility>

namespace watchpass
{

FrameProcessorNode::FrameProcessorNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  input_topic_ = declare_parameter<std::string>("input_topic", "watch_frame");
  output_topic_ = declare_parameter<std::string>("output_topic", "watch_frame_out");

  auto qos = rclcpp::SensorDataQoS();
  pub_ = create_publisher<watchpass_msgs::msg::WatchFrame>(output_topic_, qos);
  sub_ = create_subscription<watchpass_msgs::msg::WatchFrame>(
    input_topic_, qos,
    [this](const watchpass_msgs::msg::WatchFrame::ConstSharedPtr msg) {on_frame(msg);});

  RCLCPP_INFO(
    get_logger(), "%s: %s -> %s, loaned output %s",
    node_name.c_str(), input_topic_.c_str(), output_topic_.c_str(),
    pub_->can_loan_messages() ? "ENABLED (zero-copy)" : "unavailable (middleware copies)");
}

size_t FrameProcessorNode::capacity()
{
  return watchpass_msgs::msg::WatchFrame::MAX_DATA;
}

void FrameProcessorNode::copy_geometry(
  const FrameView & in, watchpass_msgs::msg::WatchFrame & out)
{
  out.width = in.width();
  out.height = in.height();
  out.step = in.step();
  out.encoding = in.encoding();
}

void FrameProcessorNode::on_frame(const watchpass_msgs::msg::WatchFrame::ConstSharedPtr msg)
{
  FrameView view(*msg);
  if (!view.valid()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "invalid input frame, dropping");
    return;
  }

  // Borrow the output buffer from the middleware (shared-memory slot when the
  // loan path is active), so the publish below is zero-copy.
  auto loaned = pub_->borrow_loaned_message();
  auto & out = loaned.get();

  // Propagate the header; the subclass owns geometry + pixels.
  out.stamp = view.stamp();
  out.seq = view.seq();

  if (!process(view, out)) {
    return;  // subclass dropped the frame
  }

  if (out.data_size > out.data.size()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "processor produced %u bytes > capacity %zu; dropping",
      out.data_size, out.data.size());
    return;
  }

  pub_->publish(std::move(loaned));
}

}  // namespace watchpass
