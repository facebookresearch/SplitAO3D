#pragma once
#include <functional>

namespace split_rendering {

class ScreenshotCaptureHelper {
 public:
  void addFrame(std::function<void()> begin_func, std::function<void()> end_func) {
    begin_frame_functions.push_back(begin_func);
    end_frame_functions.push_back(end_func);
    num_frames++;
  }

  void beginFrame() {
    if (frame_count >= num_frames)
      return;

    begin_frame_functions[frame_count]();
  }

  bool isRunning() {
    return frame_count < num_frames;
  }

  void endFrame() {
    if (frame_count >= num_frames)
      return;

    end_frame_functions[frame_count]();

    frame_count++;
  }

 private:
  std::vector<std::function<void()>> begin_frame_functions;
  std::vector<std::function<void()>> end_frame_functions;
  uint32_t frame_count = 0;
  uint32_t num_frames = 0;
};

} // namespace split_rendering