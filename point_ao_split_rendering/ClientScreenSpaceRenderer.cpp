// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

// Based on
// shared\third-party\Falcor\4.1\Falcor\Source\Samples\HelloDXR\HelloDXR.cpp

#include "ClientScreenSpaceRenderer.h"

#include <stdio.h>

static const float4 kClearColor(0.38f, 0.52f, 0.10f, 1);
//static const std::string kDefaultScene = "Arcade/Arcade.pyscene";

static const std::string kDefaultScene = "test_scenes/arcade_with_animated_things.pyscene";
// using namespace rlr_streaming;

namespace split_rendering {

void ClientScreenSpaceRenderer::onGuiRender(Gui* gui) {
  Gui::Window w(gui, "Split Client", {500, 400}, {10, 80});
  w.checkbox("AO Only", aoOnly_);
  w.checkbox("Color Only", colorOnly_);
  if (w.button("Load Scene")) {
    std::string fileName;
    if (openFileDialog(Scene::getFileExtensionFilters(), fileName)) {
      loadScene(fileName, gpFramework->getTargetFbo().get());
    }
  }


  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << smoothedLatency_ * 1000;

  w.text(std::string("Latency: ") + ss.str());

  scene_->renderUI(w);
}

void ClientScreenSpaceRenderer::loadScene(const std::string& fileName, const Fbo* targetFbo) {
  scene_ = Scene::create(fileName);
  if (!scene_)
    return;

  camera_ = scene_->getCamera();

  // Update the controllers
  float radius = length(scene_->getSceneBounds().extent());
  scene_->setCameraSpeed(radius * 0.25f);
  float nearZ = std::max(0.1f, radius / 750.0f);
  float farZ = radius * 10;
  camera_->setDepthRange(nearZ, farZ);
  camera_->setAspectRatio((float)targetFbo->getWidth() / (float)targetFbo->getHeight());

  auto typeConformances = scene_->getTypeConformances();

  rasterPass_ =
      RasterScenePass::create(scene_, "Samples/FalcorClient/CompositeClient.ps.slang", "", "main");
  rasterPass_->getProgram()->setTypeConformances(typeConformances);

  depthNormalsPrepass_ =
      RasterScenePass::create(scene_, "Samples/FalcorServer/DepthNormals.ps.slang", "", "main");
  depthNormalsPrepass_->getProgram()->setTypeConformances(typeConformances);

  computePass_ = ComputePass::create("Samples/FalcorServer/BlurPass.cs.slang");
  computePass_->getProgram()->setTypeConformances(typeConformances);

  Sampler::Desc desc;
  desc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear);
  rasterPass_->getVars()["sampler"] = Sampler::create(desc);

  smoothedLatency_ = 0.0f;

  clock_.play();
}

void ClientScreenSpaceRenderer::onLoad(RenderContext* renderContext) {
  if (gpDevice->isFeatureSupported(Device::SupportedFeatures::Raytracing) == false) {
    logFatal("Device does not support raytracing!");
  }
  
  for (EyeType eye : kAllEyes) {
    receivedMessages_[eye].resize(3);

    receivedTexture_[eye] = Texture::create2D(1, 1, ResourceFormat::R32Float);

    receivedTexIndex_[eye] = 0;
    receivedTexReady_[eye] = false;

    TCPMessageType type =
        (TCPMessageType)(((uint32_t)TCPMessageType::FullScreenAmbientOcclusionTextureLeft) + eye);

    client_.setReceiveCallback(type, [&, eye](TCPMessage&& message) {
      if ((uint32_t)message.header.type > (uint32_t)TCPMessageType::NumberOfMessageTypes)
        return;

      if (receivedMessages_[eye].empty())
        return;

      receivedMessages_[eye][BufferIndex::Receiving] = std::move(message);

      {
        std::lock_guard<std::mutex> lock(receivedTexMutex_[eye]);
        std::swap(
            receivedMessages_[eye][BufferIndex::Receiving],
            receivedMessages_[eye][BufferIndex::Ready]);
        receivedTexReady_[eye] = true;
      }
    });
  }

  loadScene(kDefaultScene, gpFramework->getTargetFbo().get());
  client_.startThreads();
}

void ClientScreenSpaceRenderer::renderDepthNormalsPrepass(
    RenderContext* renderContext,
    const Fbo::SharedPtr& targetFbo) {
  renderContext->clearFbo(targetFbo.get(), kClearColor, 1.0f, 0, FboAttachmentType::All);
  // Per-eye should already work here if the FBO has an attachment per eye.
  depthNormalsPrepass_->renderScene(renderContext, targetFbo, [&](EyeType eye) {});
}

void ClientScreenSpaceRenderer::receiveMessages() {
  FALCOR_PROFILE("receiveMessages");
  TCPMessage msg;

  while (client_.tryPopFront(msg)) {
    if (msg.header.type == TCPMessageType::LatencyMeasureMessage) {
      // We find the corresponding timestamp and simply compute the delta
      currentLatency_ = clock_.getTime() - latencyTimestamps_[msg.data[0]];
      smoothedLatency_ = currentLatency_ * movingAverageFactor_ + smoothedLatency_ * (1.0f - movingAverageFactor_);
    }
  }
}

void ClientScreenSpaceRenderer::sendMessages() {
  FALCOR_PROFILE("sendMessages");
  static int id = -1;
  id++;

  TCPMessage msg;

  if (camera_->getHmdEnabled()) {
    TCPMessage hmdMessage;
    msg.header.type = TCPMessageType::HmdStateMessage;
    msg.header.id = id;
    msg.data.resize(sizeof(hmdState_));
    std::memcpy(msg.data.data(), &hmdState_, sizeof(hmdState_));
    msg.header.size = msg.data.size();
    client_.send(msg);
  }

  msg.header.type = TCPMessageType::CameraPoseMessage;
  msg.header.id = id;
  CameraPoseData camData;
  camData.headPos = camera_->getHeadPosition();
  camData.upVec = camera_->getUpVector();
  camData.headTarget = camera_->getHeadTarget();

  msg.data.resize(sizeof(camData));
  std::memcpy(msg.data.data(), &camData, sizeof(camData));
  msg.header.size = msg.data.size();
  client_.send(msg);

  TCPMessage latencyMsg;

  latencyMsg.header.type = TCPMessageType::LatencyMeasureMessage;
  latencyTimestamps_[clock_.getFrame() % latencyTimestamps_.size()] = clock_.getTime();
  latencyMsg.header.id = id;
  latencyMsg.data.resize(1);
  latencyMsg.data[0] = (uint8_t)(clock_.getFrame() % latencyTimestamps_.size());
  latencyMsg.header.size = latencyMsg.data.size();
  client_.send(latencyMsg);
}

void ClientScreenSpaceRenderer::renderAOBlur(
    RenderContext* renderContext,
    Texture::SharedPtr& inputTex,
    Texture::SharedPtr& outputTex) {
  auto vars = computePass_->getVars();
  vars["inTex"] = inputTex;
  vars["outTex"] = outputTex;

  auto cb = vars["perFrameConstantBuffer"];

  float4 clearVal = {0.0f, 0.0f, 0.0f, 0.0f};

  renderContext->clearTexture(outputTex.get(), clearVal);
  computePass_->execute(renderContext, {inputTex->getWidth(), inputTex->getHeight(), 1});
}

void ClientScreenSpaceRenderer::onFrameRender(
    RenderContext* renderContext,
    const Fbo::SharedPtr& targetFbo) {
  renderContext->clearFbo(targetFbo.get(), kClearColor, 1.0f, 0, FboAttachmentType::All);

  clock_.tick();

  receiveMessages();
  sendMessages();

  if (scene_) {
    scene_->update(renderContext, gpFramework->getGlobalClock().getTime());

    auto rasterVars = rasterPass_->getVars();

    auto cb = rasterVars["perFrameConstantBuffer"];
    cb["aoOnly"] = aoOnly_;
    cb["colorOnly"] = colorOnly_;

    rasterPass_->renderScene(renderContext, targetFbo, [&](EyeType eye) {
      rasterVars["aoTex"] = receivedTexture_[eye];
    });
  }

  TextRenderer::render(renderContext, gpFramework->getFrameRate().getMsg(), targetFbo, {20, 20});
}

bool ClientScreenSpaceRenderer::onKeyEvent(const KeyboardEvent& keyEvent) {
  if (scene_ && scene_->onKeyEvent(keyEvent))
    return true;
  return false;
}

bool ClientScreenSpaceRenderer::onMouseEvent(const MouseEvent& mouseEvent) {
  return scene_ && scene_->onMouseEvent(mouseEvent);
}

void ClientScreenSpaceRenderer::onHmdEvent(const HmdState& hmdState) {
  if (scene_)
    scene_->onHmdEvent(hmdState);

  hmdState_ = hmdState;
}

void ClientScreenSpaceRenderer::onResizeSwapChain(uint32_t width, uint32_t height) {
  float h = (float)height;
  float w = (float)width;

  if (camera_) {
    camera_->setFocalLength(18);
    float aspectRatio = (w / h);
    camera_->setAspectRatio(aspectRatio);

    static int resChangeId = 0;
    TCPMessage resChangeMessage;
    resChangeMessage.header.type = TCPMessageType::ClientResolutionChangeMessage;
    resChangeMessage.header.id = resChangeId++;
    resChangeMessage.header.width = width;
    resChangeMessage.header.height = height;
    // This is a dummy byte, as I am not sure if the callback functions handle header-only packets.
    resChangeMessage.header.size = 1;
    resChangeMessage.data.resize(1);
    client_.send(resChangeMessage);
  }
}

} // namespace split_rendering