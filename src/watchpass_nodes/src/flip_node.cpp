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
// flip_node: example FrameProcessorNode that mirrors the frame, preserving its
// encoding. Shows a same-shape transform (copy_geometry) writing zero-copy from
// the loaned input buffer to the loaned output buffer.

#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "watchpass_framework/frame_processor_node.hpp"
#include "watchpass_framework/frame_view.hpp"

namespace watchpass_nodes
{

class FlipNode : public watchpass::FrameProcessorNode
{
public:
  explicit FlipNode(const rclcpp::NodeOptions & options)
  : watchpass::FrameProcessorNode("flip", options)
  {
    // 0 = flip vertically, 1 = horizontally, -1 = both axes.
    flip_code_ = declare_parameter<int>("flip_code", 1);
  }

protected:
  bool process(
    const watchpass::FrameView & in, watchpass_msgs::msg::WatchFrame & out) override
  {
    const cv::Mat src = in.as_mat();
    if (src.empty()) {
      return false;
    }

    copy_geometry(in, out);
    out.data_size = static_cast<uint32_t>(in.size());
    if (out.data_size > capacity()) {
      return false;
    }

    cv::Mat dst(
      static_cast<int>(out.height), static_cast<int>(out.width),
      watchpass::FrameFormat::cv_type(in.encoding()),
      out.data.data(), static_cast<size_t>(out.step));
    cv::flip(src, dst, flip_code_);
    return true;
  }

private:
  int flip_code_ = 1;
};

}  // namespace watchpass_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(watchpass_nodes::FlipNode)
