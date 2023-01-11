// (c) Meta Platforms, Inc. and its affiliates. Confidential and proprietary.

// A thread-safe queue.
// Based on https://stackoverflow.com/questions/32227321/atomic-operation-on-queuet

#pragma once

#include <mutex>
#include <queue>

namespace rlr_streaming {

template <typename T>
class AtomicQueue {
 public:
  void push(const T& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(value);
  }

  bool empty() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  bool tryPopFront(T& t) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return false;
    }
    t = std::move(queue_.front());
    queue_.pop();
    return true;
  }

 private:
  std::queue<T> queue_;
  mutable std::mutex mutex_;
};

} // namespace rlr_streaming