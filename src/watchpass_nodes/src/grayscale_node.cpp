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
// grayscale_node: example FrameProcessorNode. Converts RGB8/BGR8 -> MONO8.
//
// It demonstrates the framework's promise: the input cv::Mat aliases the loaned
// input buffer, the output cv::Mat aliases the loaned output buffer, and OpenCV
// writes the result straight from one shared buffer to the other. No frame is
// ever copied for transport.

#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "watchpass_framework/frame_processor_node.hpp"
#include "watchpass_framework/frame_view.hpp"

namespace watchpass_nodes
{

class GrayscaleNode : public watchpass::FrameProcessorNode
{
public:
  explicit GrayscaleNode(const rclcpp::NodeOptions & options)
  : watchpass::FrameProcessorNode("grayscale", options) {}

protected:
  bool process(
    const watchpass::FrameView & in, watchpass_msgs::msg::WatchFrame & out) override
  {
    const cv::Mat src = in.as_mat();
    if (src.empty()) {
      return false;
    }

    out.width = in.width();
    out.height = in.height();
    out.encoding = watchpass_msgs::msg::WatchFrame::ENCODING_MONO8;
    out.step = in.width();
    out.data_size = in.width() * in.height();
    if (out.data_size > capacity()) {
      return false;
    }

    cv::Mat dst(
      static_cast<int>(out.height), static_cast<int>(out.width), CV_8UC1,
      out.data.data(), static_cast<size_t>(out.step));

    switch (in.encoding()) {
      case watchpass_msgs::msg::WatchFrame::ENCODING_RGB8:
        cv::cvtColor(src, dst, cv::COLOR_RGB2GRAY);
        break;
      case watchpass_msgs::msg::WatchFrame::ENCODING_BGR8:
        cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
        break;
      case watchpass_msgs::msg::WatchFrame::ENCODING_MONO8:
        src.copyTo(dst);  // already grayscale
        break;
      default:
        return false;
    }
    return true;
  }
};

}  // namespace watchpass_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(watchpass_nodes::GrayscaleNode)
