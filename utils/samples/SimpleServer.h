// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#pragma once

#include <split_rendering/research_framework/Peer.h>
#include <rlr_streaming/streaming/StreamingApi.h>

namespace split_rendering {

// A very simple server implementation.
class SimpleServer : public Peer {
 public:
  SimpleServer() = delete;
  virtual ~SimpleServer();

  explicit SimpleServer(rlr_streaming::StreamingApi& streaming, int sendIntervalMs);

  // Expects a file that contains a sequence of RGB8 buffers.
  // Server will read these RGB8 images and stream them to client.
  bool setRgbFile(const std::string& rgbFile, uint32_t width, uint32_t height);

  virtual bool sendLoop() override;

  SimpleServer(const SimpleServer&) = delete;
  SimpleServer& operator=(const SimpleServer&) = delete;

 private:
  int sendIntervalMs_;

  // For sending images.
  // Assume the input RGB file to only have 3 channels.
  // WebRTC streaming library only supports 3 channels for now.
  const int kNumChannels = 3;
  size_t frameSize_ = 0;
  uint32_t frameWidth_ = 0;
  uint32_t frameHeight_ = 0;
  FILE* rgbFile_ = nullptr;
};

} // namespace split_rendering