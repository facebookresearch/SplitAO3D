/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */


// Based on
// shared\third-party\Falcor\4.1\Falcor\Source\Samples\HelloDXR\HelloDXR.h

#pragma once

#include "NetworkClient.h"

#include <Falcor.h>

using namespace Falcor;

namespace split_rendering {

class ClientScreenSpaceRenderer : public IRenderer {
 public:
  enum BufferIndex { Receiving = 0, Ready = 1, Rendering = 2 };

  explicit ClientScreenSpaceRenderer(NetworkClient& client)
      : client_(client), latencyTimestamps_(60) {}

  void onLoad(RenderContext* renderContext) override;
  void onFrameRender(RenderContext* renderContext, const Fbo::SharedPtr& targetFbo) override;
  void onResizeSwapChain(uint32_t width, uint32_t height) override;
  bool onKeyEvent(const KeyboardEvent& keyEvent) override;
  bool onMouseEvent(const MouseEvent& mouseEvent) override;
  void onHmdEvent(const HmdState& hmdState) override;
  void onGuiRender(Gui* gui) override;

 private:
  RasterScenePass::SharedPtr rasterPass_;
  Scene::SharedPtr scene_;

  RtProgram::SharedPtr raytraceProgram_ = nullptr;
  Camera::SharedPtr camera_;
  RasterScenePass::SharedPtr depthNormalsPrepass_;
  ComputePass::SharedPtr computePass_;

  bool aoOnly_ = false;
  bool colorOnly_ = false;
  // These are triple buffered (Receiving, Ready, Rendering) to enable async texture receiving
  std::vector<TCPMessage> receivedMessages_[kEyeCount];
  std::mutex receivedTexMutex_[kEyeCount];
  uint8_t receivedTexIndex_[kEyeCount];
  bool receivedTexReady_[kEyeCount];
  std::vector<double> latencyTimestamps_;
  double currentLatency_;
  float smoothedLatency_;
  const float movingAverageFactor_ = 0.3f;

  uint32_t sampleIndex_ = 0xdeadbeef;

  NetworkClient& client_;
  Clock clock_;

  Texture::SharedPtr receivedTexture_[kEyeCount];

  HmdState hmdState_;

  void renderDepthNormalsPrepass(RenderContext* context, const Fbo::SharedPtr& targetFbo);
  void receiveMessages();
  void sendMessages();
  void renderAOBlur(
      RenderContext* renderContext,
      Texture::SharedPtr& inputTex,
      Texture::SharedPtr& outputTex);
  void loadScene(const std::string& fileName, const Fbo* targetFbo);
};

} // namespace split_rendering
