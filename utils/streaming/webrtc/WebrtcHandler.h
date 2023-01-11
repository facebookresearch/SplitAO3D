// (c) Meta Platforms, Inc. and its affiliates. Confidential and proprietary.

#pragma once

#include "WebrtcPeer.h"

#include <glog/logging.h>
#include <atomic>
#include <chrono>
#include <iostream>

namespace rlr_streaming {

// This is a convenient class aimed to be subclassed by NetServer and NetworkClient within the Cloud
// Rendering System.
class WebrtcHandler : public WebrtcEventHandler,
                      public WebrtcFrameSink,
                      public WebrtcDataSink,
                      public WebrtcStatsSink {
 public:
  WebrtcHandler() {}
  virtual ~WebrtcHandler() {}

  // Whether peer connection has been established.
  bool isPeerConnectionEstablished() {
    return latestPeerConnectionEvent == WebrtcEvent::PeerConnectionConnected;
  }

  // Whether data channel has been established.
  bool isDataChannelEstablished() {
    return latestDataChannelEvent == WebrtcEvent::DataChannelOpen;
  }

  bool sendData(const void* dataBuffer, size_t bufferSize) {
    if (!webrtcPeer) {
      return false;
    }
    return webrtcPeer->sendData(dataBuffer, bufferSize);
  }

 protected:
  // Whether |webrtcPeer| is not nullptr.
  // Used as an indicator of whether WebRTC should be used for streaming (instead of TCP).
  bool hasValidPeer() {
    return webrtcPeer != nullptr;
  }

  // WebrtcEventHandler implementation.
  virtual void onEvent(rlr_streaming::WebrtcEvent event, void* data) override {
    char str[kEventStringSize] = {0};
    rlr_streaming::WebrtcPeer::eventToString(event, str, kEventStringSize);
    std::cout << "onEvent called. Event " << str << std::endl;
    if (event == rlr_streaming::WebrtcEvent::SignalingServerSignedIn && data != nullptr) {
      std::cout << "My WebRTC ID: "
                << (static_cast<rlr_streaming::SignalingServerSignedInData*>(data))->id
                << std::endl;
    }
    if (event >= rlr_streaming::WebrtcEvent::PeerConnectionNew &&
        event <= rlr_streaming::WebrtcEvent::PeerConnectionClosed) {
      latestPeerConnectionEvent = event;
    }
    if (event >= rlr_streaming::WebrtcEvent::DataChannelConnecting &&
        event <= rlr_streaming::WebrtcEvent::DataChannelClosed) {
      latestDataChannelEvent = event;
    }
  }

  // WebrtcFrameSink implementation.
  virtual void onFrame(const WebrtcFrame& /* frame */) override {}

  // WebrtcDataSink implementation.
  virtual void onData(const void* /* data_buffer */, size_t /* buffer_size */) override {}

  // WebrtcStatsSink implementation.
  virtual void onStats(const char* statsBuffer, size_t statsSize) const override {
    std::string stats(statsBuffer, statsSize);
    const auto now = std::chrono::system_clock::now();
    LOG(INFO) << __func__ << " "
              << std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count()
              << " [PeerConnectionStats] " << stats;
  };

  static constexpr size_t kEventStringSize = 100;
  std::unique_ptr<WebrtcPeer> webrtcPeer;
  std::atomic<WebrtcEvent> latestPeerConnectionEvent{WebrtcEvent::PeerConnectionNew};
  std::atomic<WebrtcEvent> latestDataChannelEvent{WebrtcEvent::DataChannelClosed};
};

} // namespace rlr_streaming
