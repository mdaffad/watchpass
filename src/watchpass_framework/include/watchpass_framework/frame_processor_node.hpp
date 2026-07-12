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
#ifndef WATCHPASS_FRAMEWORK__FRAME_PROCESSOR_NODE_HPP_
#define WATCHPASS_FRAMEWORK__FRAME_PROCESSOR_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "watchpass_framework/frame_view.hpp"
#include "watchpass_msgs/msg/watch_frame.hpp"

namespace watchpass
{

/// Base class for N-in / 1-out frame processors (grayscale, resize, crop,
/// annotate, ...). It subscribes to an input WatchFrame topic zero-copy, borrows
/// a loaned output WatchFrame from the middleware, lets the subclass fill it,
/// and publishes it zero-copy. Chain several of these in one component container
/// and every hop is a pointer move.
///
/// Subclasses implement process(). The output message is already loaned and its
/// header/seq copied from the input; the subclass sets geometry, encoding,
/// data_size and pixels. Return false to drop the frame (nothing is published).
///
/// Parameters:
///   input_topic  (string, default "watch_frame")
///   output_topic (string, default "watch_frame_out")
class FrameProcessorNode : public rclcpp::Node
{
public:
  FrameProcessorNode(const std::string & node_name, const rclcpp::NodeOptions & options);

protected:
  virtual bool process(const FrameView & in, watchpass_msgs::msg::WatchFrame & out) = 0;

  /// Copy geometry + encoding from the input into the output (for processors
  /// that keep the same frame shape and only transform pixels).
  static void copy_geometry(const FrameView & in, watchpass_msgs::msg::WatchFrame & out);

  /// Fixed payload capacity of a WatchFrame (bytes). Subclasses must keep
  /// out.data_size within this bound.
  static size_t capacity();

  const std::string & input_topic() const {return input_topic_;}
  const std::string & output_topic() const {return output_topic_;}

private:
  void on_frame(const watchpass_msgs::msg::WatchFrame::ConstSharedPtr msg);

  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Subscription<watchpass_msgs::msg::WatchFrame>::SharedPtr sub_;
  rclcpp::Publisher<watchpass_msgs::msg::WatchFrame>::SharedPtr pub_;
};

}  // namespace watchpass

#endif  // WATCHPASS_FRAMEWORK__FRAME_PROCESSOR_NODE_HPP_
