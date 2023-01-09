// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#include "KeyPress.h"

#include <glog/logging.h>

#ifdef _WIN32
#include <windows.h>
#include <atomic>
#endif

namespace split_rendering {

#ifdef _WIN32
static std::atomic<bool> g_qPressed{false};

static HHOOK g_hKeyboardHook;
LRESULT CALLBACK keyboardProc(int code, WPARAM wParam, LPARAM lParam) {
  bool isKeyDown = (wParam == WM_KEYDOWN);
  if (code == HC_ACTION && isKeyDown) {
    KBDLLHOOKSTRUCT* hookStruct = (KBDLLHOOKSTRUCT*)lParam;
    switch (hookStruct->vkCode) {
        //  https://docs.microsoft.com/en-us/windows/win32/inputdev/virtual-key-codes
      case 0x51:
        g_qPressed.store(true, std::memory_order_release);
        LOG(INFO) << "Letter 'q' pressed on keyboard";
    }
  }
  return CallNextHookEx(g_hKeyboardHook, code, wParam, lParam);
}
#endif

void KeyPress::setup() {
#ifdef _WIN32
  g_hKeyboardHook = SetWindowsHookEx(WH_KEYBOARD_LL, keyboardProc, NULL, NULL);
#else
  tcgetattr(fileno(stdin), &oldSettings_);
  newSettings_ = oldSettings_;
  newSettings_.c_lflag &= (~ICANON & ~ECHO);
  tcsetattr(fileno(stdin), TCSANOW, &newSettings_);
#endif
}

bool KeyPress::checkAndClear(char key) {
#ifdef _WIN32
  if (key == 'q' && g_qPressed.load(std::memory_order_acquire)) {
    g_qPressed.store(false, std::memory_order_release);
    return true;
  }
  return false;
#else
  fd_set set;
  struct timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  FD_ZERO(&set);
  FD_SET(fileno(stdin), &set);
  int res = select(fileno(stdin) + 1, &set, NULL, NULL, &tv);
  if (res > 0) {
    char c;
    read(fileno(stdin), &c, 1);
    if (c == key) {
      LOG(INFO) << "Letter 'q' pressed on keyboard";
      return true;
    }
  } else if (res < 0) {
    LOG(ERROR) << "Failed to listen for key press";
  }
  return false;
#endif
}

void KeyPress::reset() {
#ifdef _WIN32
  g_qPressed.store(false, std::memory_order_release);
#else
  tcsetattr(fileno(stdin), TCSANOW, &oldSettings_);
#endif
}

} // namespace split_rendering
