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

#include "NetworkServer.h"

#include <Falcor.h>
#include "Rendering/Lights/EnvMapLighting.h"

using namespace Falcor;

namespace split_rendering {

class ServerScreenSpaceRenderer : public IRenderer {
 public:

  template <typename T>
  class AtomicVar {
   public:
    void set(const T& value) {
      std::lock_guard<std::mutex> lock(mutex_);
      var_ = value;
    }

    T get() {
      std::lock_guard<std::mutex> lock(mutex_);
      return var_;
    }

   private:
    T var_;
    mutable std::mutex mutex_;
  };

  explicit ServerScreenSpaceRenderer(NetworkServer& server) : server_(server) {}

  void onLoad(RenderContext* renderContext) override;
  void onFrameRender(RenderContext* renderContext, const Fbo::SharedPtr& targetFbo) override;
  void onResizeSwapChain(uint32_t width, uint32_t height) override;
  bool onKeyEvent(const KeyboardEvent& keyEvent) override;
  bool onMouseEvent(const MouseEvent& mouseEvent) override;
  //void onHmdEvent(const HmdState& hmdState) override;
  void onGuiRender(Gui* gui) override;

 private:
  RasterScenePass::SharedPtr rasterPass_;
  RasterScenePass::SharedPtr depthNormalsPrepass_;
  Scene::SharedPtr scene_;

  ComputePass::SharedPtr computePass_;

  RtProgram::SharedPtr rtaoProgram_ = nullptr;
  Camera::SharedPtr camera_;

  bool aoOnly_ = false;
  bool colorOnly_ = false;

  // This flag enables rendering of the AO contribution in stereo, but does not open a stereo
  // window. This needs to be set, otherwise stereo rendering is completely disabled. To view the
  // server output in stereo, enable the stereo flag in the falcor config settings in
  // ServerMain.cpp
  bool stereoServer_ = true;
  int aoKernel_ = 3;
  int aoSamples_ = 32;
  float aoRadius_ = 0.1f;
  RtProgramVars::SharedPtr rtaoVars_;
  Texture::SharedPtr texAO_[kEyeCount];
  Texture::SharedPtr texBlurredAO_[kEyeCount];
  Fbo::SharedPtr rtaoFBO_;
  Fbo::SharedPtr blurredAOFBO_;
  GraphicsVars::SharedPtr perEyeVars_;

  uint32_t sampleIndex_ = 0xdeadbeef;

  NetworkServer& server_;

  AtomicVar<uint8_t> newestLatencyId_;
  uint8_t currentLatencyId_;
  EnvMapLighting::SharedPtr envMapLighting_;

  void sendMessages();
  void receiveMessages();

  void setPerFrameVars(const Fbo* targetFbo, EyeType eye);
  void renderRT(RenderContext* renderContext, const Fbo* targetFbo);
  void renderAOBlur(RenderContext* renderContext, const Fbo* inputFbo, const Fbo* targetFbo);
  void loadScene(const std::string& filename, const Fbo* targetFbo);
};

} // namespace split_rendering
