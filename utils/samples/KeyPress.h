// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#pragma once

#ifndef _WIN32
#include <termios.h>
#endif

namespace split_rendering {

class KeyPress {
 public:
  KeyPress() = default;
  virtual ~KeyPress() = default;

  // Setup to capture key press on keyboard.
  void setup();

  // Return whether |key| was pressed, then clear state about that key.
  // Currently only support checking key 'q' on Windows.
  bool checkAndClear(char key);

  // Reset state, e.g., reset original terminal setting on Linux.
  void reset();

  KeyPress(const KeyPress&) = delete;
  KeyPress& operator=(const KeyPress&) = delete;

 private:
#ifndef _WIN32
  // For capturing keyboard press on Linux
  // http://www.cplusplus.com/forum/unices/11910/
  struct termios oldSettings_;
  struct termios newSettings_;
#endif
};

} // namespace split_rendering
