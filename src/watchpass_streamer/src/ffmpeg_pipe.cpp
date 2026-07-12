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
#include "watchpass_streamer/ffmpeg_pipe.hpp"

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace watchpass_streamer
{

FfmpegPipe::~FfmpegPipe()
{
  stop();
}

bool FfmpegPipe::start(
  const std::string & ffmpeg_path,
  const FfmpegInput & input,
  const std::vector<std::string> & output_args)
{
  if (running()) {
    return false;  // already started; caller must stop() first
  }

  int fds[2];
  if (pipe(fds) != 0) {
    std::fprintf(stderr, "[ffmpeg_pipe] pipe() failed: %s\n", std::strerror(errno));
    return false;
  }
  const int read_fd = fds[0];
  const int write_fd = fds[1];

  // Assemble argv:
  //   ffmpeg -hide_banner -loglevel warning
  //          -f rawvideo -pix_fmt <fmt> -s <w>x<h> -r <fps> -i pipe:0
  //          <output_args...>
  const std::string size = std::to_string(input.width) + "x" + std::to_string(input.height);
  const std::string rate = std::to_string(input.fps);

  std::vector<std::string> args = {
    ffmpeg_path,
    "-hide_banner", "-loglevel", "warning",
    "-f", "rawvideo",
    "-pix_fmt", input.pixel_format,
    "-s", size,
    "-r", rate,
    "-i", "pipe:0",
  };
  args.insert(args.end(), output_args.begin(), output_args.end());

  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (auto & a : args) {
    argv.push_back(const_cast<char *>(a.c_str()));
  }
  argv.push_back(nullptr);

  const pid_t pid = fork();
  if (pid < 0) {
    std::fprintf(stderr, "[ffmpeg_pipe] fork() failed: %s\n", std::strerror(errno));
    close(read_fd);
    close(write_fd);
    return false;
  }

  if (pid == 0) {
    // --- child ---
    // Wire the pipe read end to stdin, then exec ffmpeg.
    if (dup2(read_fd, STDIN_FILENO) < 0) {
      std::perror("[ffmpeg_pipe] dup2");
      _exit(127);
    }
    close(read_fd);
    close(write_fd);
    execvp(argv[0], argv.data());
    // Only reached if exec failed.
    std::fprintf(stderr, "[ffmpeg_pipe] execvp(%s) failed: %s\n", argv[0], std::strerror(errno));
    _exit(127);
  }

  // --- parent ---
  close(read_fd);
  // Don't die with SIGPIPE if ffmpeg exits; write_frame() checks the return code.
  ::signal(SIGPIPE, SIG_IGN);

  pid_ = pid;
  write_fd_ = write_fd;
  input_ = input;
  return true;
}

bool FfmpegPipe::write_frame(const uint8_t * data, size_t size)
{
  if (write_fd_ < 0) {
    return false;
  }
  size_t written = 0;
  while (written < size) {
    const ssize_t n = ::write(write_fd_, data + written, size - written);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      // EPIPE etc: ffmpeg is gone.
      return false;
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

void FfmpegPipe::stop()
{
  if (write_fd_ >= 0) {
    close(write_fd_);  // EOF tells ffmpeg to flush and exit
    write_fd_ = -1;
  }
  if (pid_ > 0) {
    // Give ffmpeg a moment to flush on EOF, then reap it.
    int status = 0;
    for (int i = 0; i < 50; ++i) {
      const pid_t r = waitpid(pid_, &status, WNOHANG);
      if (r == pid_ || r < 0) {
        pid_ = -1;
        return;
      }
      usleep(20 * 1000);  // 20 ms
    }
    // Still alive after ~1 s: terminate.
    kill(pid_, SIGTERM);
    waitpid(pid_, &status, 0);
    pid_ = -1;
  }
}

}  // namespace watchpass_streamer
