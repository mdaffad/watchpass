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
#include "watchpass_streamer/v4l2_source.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>
#include <utility>

#include "watchpass_msgs/msg/watch_frame.hpp"

namespace watchpass_streamer
{

namespace
{
using WatchFrame = watchpass_msgs::msg::WatchFrame;

// Retry ioctl across EINTR, the standard V4L2 idiom. The request type is
// unsigned long because that is what the ioctl() prototype mandates.
int xioctl(int fd, unsigned long request, void * arg)  // NOLINT(runtime/int)
{
  int r;
  do {
    r = ioctl(fd, request, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

// WatchFrame encoding -> V4L2 fourcc, for the directly-mappable formats.
uint32_t encoding_to_v4l2(uint8_t encoding)
{
  switch (encoding) {
    case WatchFrame::ENCODING_RGB8: return V4L2_PIX_FMT_RGB24;
    case WatchFrame::ENCODING_BGR8: return V4L2_PIX_FMT_BGR24;
    case WatchFrame::ENCODING_MONO8: return V4L2_PIX_FMT_GREY;
    default: return 0;
  }
}

bool v4l2_to_encoding(uint32_t fourcc, uint8_t & out)
{
  switch (fourcc) {
    case V4L2_PIX_FMT_RGB24: out = WatchFrame::ENCODING_RGB8; return true;
    case V4L2_PIX_FMT_BGR24: out = WatchFrame::ENCODING_BGR8; return true;
    case V4L2_PIX_FMT_GREY: out = WatchFrame::ENCODING_MONO8; return true;
    default: return false;
  }
}

std::string fourcc_str(uint32_t f)
{
  return {
    static_cast<char>(f & 0xff), static_cast<char>((f >> 8) & 0xff),
    static_cast<char>((f >> 16) & 0xff), static_cast<char>((f >> 24) & 0xff)};
}
}  // namespace

V4l2Source::V4l2Source(rclcpp::Node & node, const Config & config)
: node_(node), config_(config) {}

V4l2Source::~V4l2Source()
{
  stop();
}

void V4l2Source::start(Callback cb)
{
  cb_ = std::move(cb);
  if (!open_device() || !negotiate_format() || !request_buffers() || !start_streaming()) {
    teardown();
    return;
  }
  set_frame_rate();  // best-effort, non-fatal
  running_ = true;
  thread_ = std::thread(&V4l2Source::capture_loop, this);
  RCLCPP_INFO(
    node_.get_logger(),
    "source=v4l2: capturing %s at %ux%u %s (%u buffers)",
    config_.device.c_str(), negotiated_width_, negotiated_height_,
    fourcc_str(encoding_to_v4l2(negotiated_encoding_)).c_str(), config_.buffer_count);
}

void V4l2Source::stop()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
  teardown();
}

bool V4l2Source::open_device()
{
  fd_ = ::open(config_.device.c_str(), O_RDWR | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(
      node_.get_logger(), "cannot open %s: %s", config_.device.c_str(), std::strerror(errno));
    return false;
  }
  v4l2_capability cap{};
  if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
    RCLCPP_ERROR(node_.get_logger(), "VIDIOC_QUERYCAP failed: %s", std::strerror(errno));
    return false;
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    RCLCPP_ERROR(node_.get_logger(), "%s is not a video capture device", config_.device.c_str());
    return false;
  }
  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    RCLCPP_ERROR(node_.get_logger(), "%s does not support streaming I/O", config_.device.c_str());
    return false;
  }
  return true;
}

bool V4l2Source::negotiate_format()
{
  const uint32_t fourcc = encoding_to_v4l2(config_.encoding);
  if (fourcc == 0) {
    RCLCPP_ERROR(node_.get_logger(), "requested encoding %u has no V4L2 mapping", config_.encoding);
    return false;
  }

  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = config_.width;
  fmt.fmt.pix.height = config_.height;
  fmt.fmt.pix.pixelformat = fourcc;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
    RCLCPP_ERROR(node_.get_logger(), "VIDIOC_S_FMT failed: %s", std::strerror(errno));
    return false;
  }

  // The driver may have adjusted the format; honor what it actually gave us.
  if (!v4l2_to_encoding(fmt.fmt.pix.pixelformat, negotiated_encoding_)) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "device negotiated unsupported format '%s'; only RGB24/BGR24/GREY map to "
      "WatchFrame. Cameras emitting YUYV/MJPEG need a conversion step.",
      fourcc_str(fmt.fmt.pix.pixelformat).c_str());
    return false;
  }
  negotiated_width_ = fmt.fmt.pix.width;
  negotiated_height_ = fmt.fmt.pix.height;
  negotiated_step_ = fmt.fmt.pix.bytesperline;
  return true;
}

bool V4l2Source::set_frame_rate()
{
  if (config_.fps <= 0.0) {
    return true;
  }
  v4l2_streamparm parm{};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(config_.fps);
  if (xioctl(fd_, VIDIOC_S_PARM, &parm) < 0) {
    RCLCPP_WARN(
      node_.get_logger(), "VIDIOC_S_PARM (fps) not honored: %s", std::strerror(errno));
    return false;
  }
  return true;
}

bool V4l2Source::request_buffers()
{
  v4l2_requestbuffers req{};
  req.count = config_.buffer_count;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
    RCLCPP_ERROR(node_.get_logger(), "VIDIOC_REQBUFS failed: %s", std::strerror(errno));
    return false;
  }
  if (req.count < 2) {
    RCLCPP_ERROR(node_.get_logger(), "insufficient device buffer memory");
    return false;
  }

  buffers_.resize(req.count);
  for (uint32_t i = 0; i < req.count; ++i) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      RCLCPP_ERROR(node_.get_logger(), "VIDIOC_QUERYBUF failed: %s", std::strerror(errno));
      return false;
    }
    buffers_[i].length = buf.length;
    buffers_[i].start = mmap(
      nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
    if (buffers_[i].start == MAP_FAILED) {
      buffers_[i].start = nullptr;
      RCLCPP_ERROR(node_.get_logger(), "mmap failed: %s", std::strerror(errno));
      return false;
    }
    // Queue the buffer for the driver to fill.
    if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
      RCLCPP_ERROR(node_.get_logger(), "VIDIOC_QBUF failed: %s", std::strerror(errno));
      return false;
    }
  }
  return true;
}

bool V4l2Source::start_streaming()
{
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    RCLCPP_ERROR(node_.get_logger(), "VIDIOC_STREAMON failed: %s", std::strerror(errno));
    return false;
  }
  return true;
}

void V4l2Source::capture_loop()
{
  while (running_) {
    pollfd pfd{fd_, POLLIN, 0};
    const int pr = poll(&pfd, 1, 500);  // 500 ms timeout so we can observe running_
    if (pr <= 0) {
      if (pr < 0 && errno != EINTR) {
        RCLCPP_WARN_THROTTLE(
          node_.get_logger(), *node_.get_clock(), 5000, "poll failed: %s", std::strerror(errno));
      }
      continue;
    }

    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
      if (errno == EAGAIN) {
        continue;
      }
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000, "VIDIOC_DQBUF failed: %s",
        std::strerror(errno));
      continue;
    }

    if (buf.index < buffers_.size() && cb_) {
      watchpass::RawFrame f;
      f.width = negotiated_width_;
      f.height = negotiated_height_;
      f.step = negotiated_step_;
      f.encoding = negotiated_encoding_;
      f.data = static_cast<const uint8_t *>(buffers_[buf.index].start);
      f.size = buf.bytesused ? buf.bytesused : buffers_[buf.index].length;
      f.stamp = node_.get_clock()->now();
      f.has_stamp = true;
      cb_(f);  // one copy into a loaned WatchFrame happens inside the callback
    }

    // Return the buffer to the driver.
    if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000, "VIDIOC_QBUF failed: %s",
        std::strerror(errno));
    }
  }
}

void V4l2Source::teardown()
{
  if (fd_ >= 0) {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_, VIDIOC_STREAMOFF, &type);
  }
  for (auto & b : buffers_) {
    if (b.start) {
      munmap(b.start, b.length);
      b.start = nullptr;
    }
  }
  buffers_.clear();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace watchpass_streamer
