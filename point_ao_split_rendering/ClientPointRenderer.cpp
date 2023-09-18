/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */


// Based on
// shared\third-party\Falcor\4.1\Falcor\Source\Samples\HelloDXR\HelloDXR.cpp

#include "ClientPointRenderer.h"

#include <stdio.h>

static const float4 kClearColor(0.38f, 0.52f, 0.10f, 1);
// static const std::string kDefaultScene = "Arcade/Arcade.pyscene";
static const std::string kDefaultScene = "test_scenes/arcade_with_animated_things.pyscene";
// static const std::string kDefaultScene = "test_scenes/arcade_with_animated_things.pyscene";
// static const std::string kDefaultScene =
//    "test_scenes/arcade_with_animated_things_fewer_objects.pyscene";
//
// using namespace rlr_streaming;

namespace split_rendering {

void ClientPointRenderer::onGuiRender(Gui* gui) {
  Gui::Window w(gui, "Hello DXR Settings", {300, 600}, {10, 30});
  w.checkbox("AO Only", aoOnly_);
  w.checkbox("Color Only", colorOnly_);

  w.slider("Int. Radius Mult", interpolationRadiusFactor_, 0.1f, 5.0f);
  w.slider("cos(N) threshold", cosNormalThreshold_, 0.0f, 1.0f);
  w.slider("cos(delta) threshold", cosDeltaThreshold_, 0.0f, 1.0f);

  if (w.button("Load Scene")) {
    std::string filename;
    if (openFileDialog(Scene::getFileExtensionFilters(), filename)) {
      loadScene(filename, gpFramework->getTargetFbo().get());
    }
  }

  {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << smoothedLatency_ * 1000;
    w.text(std::string("Latency: ") + ss.str());
  }

  if (!dataRateStats_.empty())
    w.text(std::string("Mb/s: ") + std::to_string(dataRateStats_.back().mbps));

  w.text(std::string("Current time: ") + std::to_string(gpFramework->getGlobalClock().getTime()));
  w.text(std::string("Last server time: ") + std::to_string(lastServerTime_));

  scene_->renderUI(w);
}

void ClientPointRenderer::loadScene(const std::string& filename, const Fbo* targetFbo) {
  scene_ = Scene::create(filename);
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

  rasterPass_ = RasterScenePass::create(
      scene_, "Samples/FalcorServer/ClientCompositePointAO.ps.slang", "", "main");
  rasterPass_->getProgram()->setTypeConformances(typeConformances);
  rasterPass_->getProgram()->setGenerateDebugInfoEnabled(true);

  auto depthDesc = DepthStencilState::Desc();

  // We use this to enable a depth prepass, otherwise geometric complexity hurts the AO
  // reconstruction performance quite a bit.
  depthDesc.setDepthEnabled(true);
  depthDesc.setDepthFunc(DepthStencilState::Func::LessEqual);

  rasterPass_->getState()->setDepthStencilState(DepthStencilState::create(depthDesc));

  pointHashReceiver_.init();

  smoothedLatency_ = 0.0f;

  clock_.play();
}

void ClientPointRenderer::onLoad(RenderContext* renderContext) {
  if (gpDevice->isFeatureSupported(Device::SupportedFeatures::Raytracing) == false) {
    logFatal("Device does not support raytracing!");
  }

  loadScene(kDefaultScene, gpFramework->getTargetFbo().get());

  client_.startThreads();
}

void ClientPointRenderer::receiveMessages(RenderContext* renderContext) {
  FALCOR_PROFILE("receiveMessages");
  TCPMessage msg;

  bool msgReceived = false;

  while (client_.tryPopFront(msg)) {
    totalBytesReceived_ += sizeof(msg) + msg.data.size();
    if (msg.header.type == TCPMessageType::LatencyMeasureMessage) {
      double currentLatency = gpFramework->getGlobalClock().getTime() - msg.header.timestamp;
      smoothedLatency_ =
          currentLatency * movingAverageFactor_ + smoothedLatency_ * (1.0f - movingAverageFactor_);
      latencyMeasureStart_ = -1.0;
    } else {
      msgReceived = true;
      pointHashReceiver_.receive(msg, renderContext);

      if (msg.header.type == TCPMessageType::PAOPointCellUpdate ||
          msg.header.type == TCPMessageType::PAOHashUpdate) {
        lastServerTime_ = msg.header.timestamp;
      }

      if (msg.header.type == TCPMessageType::PAOEndOfInit) {
        TCPMessage msg;
        msg.header.type = TCPMessageType::PAOEndOfInit;
        msg.header.id = 0;

        msg.header.size = 0;
        msg.header.width = 0;
        msg.header.height = 0;
        client_.send(msg);

        gpFramework->getGlobalClock().setFrame(0);
        gpFramework->getGlobalClock().setTime(0.0);
        dataRateStats_.clear();
        bytesReceivedTimestamp_ = 0.0;
      }
    }
  }

  // This is for profiling so that Falcor always has this function in the profiler, otherwise it
  // doesn't include it in the logs...
  if (!msgReceived) {
    pointHashReceiver_.receive(msg, renderContext);
  }
}

void ClientPointRenderer::sendMessages() {
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

  if (latencyMeasureStart_ < 0) {
    TCPMessage latencyMsg;

    latencyMsg.header.type = TCPMessageType::LatencyMeasureMessage;
    latencyMsg.header.timestamp = gpFramework->getGlobalClock().getTime();
    latencyMsg.header.id = id;
    latencyMsg.data.resize(sizeof(double));
    latencyMeasureStart_ = gpFramework->getGlobalClock().getTime();

    std::memcpy(latencyMsg.data.data(), &latencyMeasureStart_, sizeof(double));
    latencyMsg.header.size = latencyMsg.data.size();
    client_.send(latencyMsg);
  }
}

void ClientPointRenderer::onFrameRender(
    RenderContext* renderContext,
    const Fbo::SharedPtr& targetFbo) {
  renderContext->clearFbo(targetFbo.get(), kClearColor, 1.0f, 0, FboAttachmentType::All);

  clock_.tick();

  receiveMessages(renderContext);

  if ((gpFramework->getGlobalClock().getTime() - bytesReceivedTimestamp_) >= 1.0) {
    bytesReceivedTimestamp_ = gpFramework->getGlobalClock().getTime();
    dataRateStats_.push_back({kToMegabit * totalBytesReceived_, (float)bytesReceivedTimestamp_});
    totalBytesReceived_ = 0;
  }

  if (!pointHashReceiver_.isInitialized())
    return;

  if (scene_) {
    // TODO/HACK: We hackily use the previous frame's transformations to perform the point sampling
    // correctly
    //            Ideally, this should be done only when new server data arrives, but this way we
    //            don't need to modify Falcor.
    scene_->update(renderContext, lastServerTime_);
    scene_->update(renderContext, gpFramework->getGlobalClock().getTime());

    auto rasterVars = rasterPass_->getVars();
    // Update env map lighting
    const auto& envMap = scene_->getEnvMap();
    if (envMap && (!envMapLighting_ || envMapLighting_->getEnvMap() != envMap)) {
      envMapLighting_ = EnvMapLighting::create(renderContext, envMap);
      envMapLighting_->setShaderData(rasterVars["envMapLighting"]);
      rasterPass_->getProgram()->addDefine("_USE_ENV_MAP");
    } else if (!envMap) {
      envMapLighting_ = nullptr;
      rasterPass_->getProgram()->removeDefine("_USE_ENV_MAP");
    }

    auto constantBuffer = rasterVars["perFrameConstantBuffer"];

    // Depth prepass
    {
      FALCOR_PROFILE("Depth Prepass");
      constantBuffer["depthPrepass"] = true;
      rasterPass_->renderScene(renderContext, targetFbo, [&](EyeType eye) {});
    }

    constantBuffer["depthPrepass"] = false;
    constantBuffer["aoOnly"] = aoOnly_;
    constantBuffer["colorOnly"] = colorOnly_;
    constantBuffer["interpolationRadiusFactor"] = interpolationRadiusFactor_;
    constantBuffer["cosNormalThreshold"] = cosNormalThreshold_;
    constantBuffer["cosDeltaThreshold"] = cosDeltaThreshold_;
    rasterVars["compressedClientAOPoints"] = pointHashReceiver_.getGPUCompressedClientPointCells();
    rasterVars["instanceToDiskRadius"] = pointHashReceiver_.getGPUDiskRadiusPerInstance();
    rasterVars["serverInstanceHashInfo"] = pointHashReceiver_.getGPUInstanceHashInfo();
    rasterVars["instancePointInfo"] = pointHashReceiver_.getGPUInstancePointInfo();
    rasterVars["serverHashToPointCell"] = pointHashReceiver_.getGPUHashToPointCell();

    rasterPass_->renderScene(renderContext, targetFbo, [&](EyeType eye) {});
  }

  sendMessages();
  TextRenderer::render(renderContext, gpFramework->getFrameRate().getMsg(), targetFbo, {20, 20});
}

bool ClientPointRenderer::onKeyEvent(const KeyboardEvent& keyEvent) {
  if (keyEvent.type == KeyboardEvent::Type::KeyReleased) {
    if (keyEvent.key == KeyboardEvent::Key::Minus) {
      // Write out the network rate as .csv
      // TODO: abstract away

      std::fstream csv;
      csv.open(
          "data_rate_" + std::to_string(gpFramework->getGlobalClock().getTime()) + ".csv",
          std::ios::out);
      if (csv.is_open()) {
        csv << "Time Stamp,Mbit/s\n";
        for (const auto& drs : dataRateStats_)
          csv << drs.timeStamp << "," << drs.mbps << "\n";
      }
    }
  }

  if (scene_ && scene_->onKeyEvent(keyEvent))
    return true;
  return false;
}

bool ClientPointRenderer::onMouseEvent(const MouseEvent& mouseEvent) {
  return scene_ && scene_->onMouseEvent(mouseEvent);
}

void ClientPointRenderer::onHmdEvent(const HmdState& hmdState) {
  if (scene_)
    scene_->onHmdEvent(hmdState);

  hmdState_ = hmdState;
}

void ClientPointRenderer::onResizeSwapChain(uint32_t width, uint32_t height) {
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
