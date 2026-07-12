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
#ifndef WATCHPASS_FRAMEWORK__FRAME_CONSUMER_NODE_HPP_
#define WATCHPASS_FRAMEWORK__FRAME_CONSUMER_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "watchpass_framework/frame_view.hpp"
#include "watchpass_msgs/msg/watch_frame.hpp"

namespace watchpass
{

/// Base class for terminal frame sinks (encoders, recorders, detectors that
/// only report, ...). Subscribes to a WatchFrame topic zero-copy and hands each
/// frame to consume(). The subscriber never copies pixels: on the same host the
/// frame is read straight from the shared buffer it was published into.
///
/// Subclasses implement consume() and typically declare their own parameters.
///
/// Parameters:
///   input_topic (string, default "watch_frame")
class FrameConsumerNode : public rclcpp::Node
{
public:
  FrameConsumerNode(const std::string & node_name, const rclcpp::NodeOptions & options);

protected:
  /// Handle one frame. Called on the executor thread; keep it non-blocking.
  virtual void consume(const FrameView & frame) = 0;

  const std::string & input_topic() const {return input_topic_;}

private:
  void on_frame(const watchpass_msgs::msg::WatchFrame::ConstSharedPtr msg);

  std::string input_topic_;
  rclcpp::Subscription<watchpass_msgs::msg::WatchFrame>::SharedPtr sub_;
};

}  // namespace watchpass

#endif  // WATCHPASS_FRAMEWORK__FRAME_CONSUMER_NODE_HPP_
