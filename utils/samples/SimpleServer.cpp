// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#include "SimpleServer.h"

#include <glog/logging.h>

#include <chrono>
#include <thread>

using rlr_streaming::Event;
using rlr_streaming::Message;
using rlr_streaming::MessageMetadata;
using rlr_streaming::MessageType;
using rlr_streaming::StreamingApi;

namespace split_rendering {

SimpleServer::SimpleServer(StreamingApi& streaming, int sendIntervalMs)
    : Peer(streaming), sendIntervalMs_(sendIntervalMs) {
  setName("SimpleServer");
  setOnEventCallback();
  setOnMessageCallback();
}

SimpleServer::~SimpleServer() {
  if (rgbFile_) {
    fclose(rgbFile_);
  }
}

bool SimpleServer::setRgbFile(const std::string& rgbFile, uint32_t width, uint32_t height) {
  if (rgbFile.empty()) {
    LOG(ERROR) << "rgbFile is empty";
    return false;
  }
  if (width == 0 || height == 0) {
    LOG(ERROR) << "Invalid width or height";
    return false;
  }
  rgbFile_ = fopen(rgbFile.c_str(), "rb");
  if (!rgbFile_) {
    LOG(ERROR) << "Failed to open rgbFile " << rgbFile;
    return false;
  }
  frameWidth_ = width;
  frameHeight_ = height;
  frameSize_ = width * height * kNumChannels;
  return true;
}

bool SimpleServer::sendLoop() {
  LOG(INFO) << name() << " send loop started";

  waitForChannelsOpen();

  std::vector<uint8_t> frame;
  if (frameSize_ > 0) {
    frame.resize(frameSize_);
  }
  while (!stopRequested()) {
    uint64_t sendStartMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();

    // Send a text message.
    static int i = 0;
    std::string text = std::to_string(i) + " th message from " + name();
    Message message(text.data(), text.size());
    MessageMetadata metadata;
    metadata.type = MessageType::Binary;
    LOG(INFO) << name() << " sending text message: " << text;
    if (!send(message, metadata)) {
      LOG(ERROR) << name() << " failed to send text message";
    }
    i++;

    // Send a video frame message.
    if (rgbFile_) {
      if (!fread(frame.data(), 1, frameSize_, rgbFile_)) {
        rewind(rgbFile_);
        if (!fread(frame.data(), 1, frameSize_, rgbFile_)) {
          LOG(ERROR) << "Failed to read frame file";
          return false;
        }
      }
      Message message(frame.data(), frame.size());
      MessageMetadata metadata;
      static int id = 0;
      metadata.id = id;
      metadata.type = MessageType::ImageRgb8;
      metadata.width = frameWidth_;
      metadata.height = frameHeight_;
      LOG(INFO) << name() << " sending a video frame with id " << id;
      if (!send(message, metadata)) {
        LOG(ERROR) << name() << " failed to send video frame";
      }
      id++;
    }

    uint64_t sendEndMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
    uint64_t sleepMs = sendIntervalMs_ - (sendEndMs - sendStartMs);
    if (sleepMs > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
    }
  }

  LOG(INFO) << name() << " send loop finished";

  return true;
}

} // namespace split_rendering