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

#include "ClientPointHashReceiver.h"
#include "Rendering/Lights/EnvMapLighting.h"

using namespace Falcor;

namespace split_rendering {

class ClientPointRenderer : public IRenderer {
 public:
  enum BufferIndex { Receiving = 0, Ready = 1, Rendering = 2 };

  explicit ClientPointRenderer(NetworkClient& client)
      : client_(client) {}

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

  Camera::SharedPtr camera_;

  bool aoOnly_ = false;
  bool colorOnly_ = false;

  float smoothedLatency_;
  const float movingAverageFactor_ = 0.3f;

  uint32_t sampleIndex_ = 0xdeadbeef;

  NetworkClient& client_;
  Clock clock_;
  RenderContext::SharedPtr copyContext_;

  float interpolationRadiusFactor_ = 1.0f;
  float cosNormalThreshold_ = 0.2f;
  float cosDeltaThreshold_ = 0.1f;
  double latencyMeasureStart_ = -1.0;
  uint64_t totalBytesReceived_ = 0;
  double bytesReceivedTimestamp_ = 0.0;
  float lastServerTime_ = 0.0f;
  constexpr static const float kToMegabit = 8.0f * 0.000001f;

  struct DataRateStats {
    float mbps;
    float timeStamp;
  };
  std::vector<DataRateStats> dataRateStats_;
  
  EnvMapLighting::SharedPtr envMapLighting_;

  HmdState hmdState_;

  ClientPointHashReceiver pointHashReceiver_;

  void receiveMessages(RenderContext* renderContext);
  void sendMessages();
  void loadScene(const std::string& filename, const Fbo* targetFbo);
};

} // namespace split_rendering
