// (c) Meta Platforms, Inc. and its affiliates. Confidential and proprietary.

#pragma once

#include <mutex>

#include "StreamingApi.h"
#include "webrtc/WebrtcHandler.h"
#include "webrtc/WebrtcPeer.h"

namespace rlr_streaming {

// WebRTC implementation of the streaming API
class WebrtcStreamingApi final : public StreamingApi, public WebrtcHandler {
 public:
  WebrtcStreamingApi() = default;
  virtual ~WebrtcStreamingApi() {}

  // This will initialize WebRTC and starts the signaling process i.e., connecting to the signaling
  // server. Depends on the value in |init|, it might also kick off P2P connection process via the
  // signaling server.
  // TODO(fangy): Consider separate these operations in to a init() and connect() calls. Which will
  // require the change to WebrtcPeer.h API.
  // |init| is used to initialize WebrtcPeer.
  bool init(const WebrtcPeerInit& init);

  bool send(const Message& message, const MessageMetadata& metadata) override;

  void stop() override;

  // WebrtcEventHandler implementation.
  void onEvent(WebrtcEvent event, void* data) override;

  // WebrtcFrameSink implementation.
  void onFrame(const WebrtcFrame& frame) override;

  // WebrtcDataSink implementation.
  void onData(const void* dataBuffer, size_t bufferSize) override;

  WebrtcStreamingApi(const WebrtcStreamingApi&) = delete;
  WebrtcStreamingApi& operator=(const WebrtcStreamingApi&) = delete;

 private:
  WebrtcPeerInit webrtcPeerInit_;
  std::unique_ptr<WebrtcPeer> webrtcPeer_;
  mutable std::mutex webrtcPeerMutex_;
};

} // namespace rlr_streaming