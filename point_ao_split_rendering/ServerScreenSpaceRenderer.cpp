// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

// Based on
// shared\third-party\Falcor\4.1\Falcor\Source\Samples\HelloDXR\HelloDXR.cpp

#include "ServerScreenSpaceRenderer.h"
#include "BinaryMessageType.h"

//#include <glog/logging.h>

using namespace rlr_streaming;

namespace split_rendering {

static const float4 kClearColor(0.38f, 0.52f, 0.10f, 1);
static const std::string kDefaultScene = "Arcade/Arcade.pyscene";

void ServerScreenSpaceRenderer::onGuiRender(Gui* gui) {
  Gui::Window w(gui, "Hello DXR Settings", {300, 400}, {10, 80});

  w.checkbox("AO Only", aoOnly_);
  w.checkbox("Color Only", colorOnly_);
  w.slider("AO samples", aoSamples_, 1, 256);
  w.slider("AO radius", aoRadius_, 0.001f, 5.0f);
  w.slider("AO filter kernel", aoKernel_, 1, 10);
  if (w.button("Load Scene")) {
    std::string filename;
    if (openFileDialog(Scene::getFileExtensionFilters(), filename)) {
      loadScene(filename, gpFramework->getTargetFbo().get());
    }
  }

  if (scene_)
    scene_->renderUI(w);
}

void ServerScreenSpaceRenderer::loadScene(const std::string& filename, const Fbo* targetFbo) {
  scene_ = Scene::create(filename);
  if (!scene_)
    return;

  camera_ = scene_->getCamera();

  auto typeConformances = scene_->getTypeConformances();
  // Update the controllers
  float radius = length(scene_->getSceneBounds().extent());
  scene_->setCameraSpeed(radius * 0.25f);
  float nearZ = std::max(0.1f, radius / 750.0f);
  float farZ = radius * 10;
  camera_->setDepthRange(nearZ, farZ);
  camera_->setAspectRatio((float)targetFbo->getWidth() / (float)targetFbo->getHeight());

  rasterPass_ =
      RasterScenePass::create(scene_, "Samples/FalcorServer/FinalComposite.ps.slang", "", "main");
  rasterPass_->getProgram()->setTypeConformances(typeConformances);

  depthNormalsPrepass_ =
      RasterScenePass::create(scene_, "Samples/FalcorServer/DepthNormals.ps.slang", "", "main");
  depthNormalsPrepass_->getProgram()->setTypeConformances(typeConformances);

  Sampler::Desc desc;
  desc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear);
  rasterPass_->getVars()["mainSampler"] = Sampler::create(desc);

  computePass_ = ComputePass::create("Samples/FalcorServer/BlurPass.cs.slang");

  RtProgram::Desc rtProgDesc;
  rtProgDesc.addShaderLibrary("Samples/FalcorServer/RTAO.rt.slang");

  // 1 for calling TraceRay from RayGen, 1 for calling it from the primary-ray ClosestHit
  // shader for reflections, 1 for reflection ray tracing a shadow ray
  rtProgDesc.setMaxTraceRecursionDepth(2);

  // The largest ray payload struct (PrimaryRayData) is 16 bytes. The payload size should
  // be set as small as possible for maximum performance.
  rtProgDesc.setMaxPayloadSize(16);

  RtBindingTable::SharedPtr sbt = RtBindingTable::create(2, 2, scene_->getGeometryCount());
  sbt->setRayGen(rtProgDesc.addRayGen("rayGen"));
  sbt->setMiss(0, rtProgDesc.addMiss("primaryMiss"));
  sbt->setMiss(1, rtProgDesc.addMiss("aoMiss"));
  auto primary = rtProgDesc.addHitGroup("primaryClosestHit", "primaryAnyHit");
  auto ao = rtProgDesc.addHitGroup("aoClosestHit", "");
  sbt->setHitGroup(0, scene_->getGeometryIDs(Scene::GeometryType::TriangleMesh), primary);
  sbt->setHitGroup(1, scene_->getGeometryIDs(Scene::GeometryType::TriangleMesh), ao);

  rtaoProgram_ = RtProgram::create(rtProgDesc, scene_->getSceneDefines());
  rtaoProgram_->setTypeConformances(typeConformances);
  rtaoVars_ = RtProgramVars::create(rtaoProgram_, sbt);
}

void ServerScreenSpaceRenderer::onLoad(RenderContext* renderContext) {
  if (gpDevice->isFeatureSupported(Device::SupportedFeatures::Raytracing) == false) {
    logFatal("Device does not support raytracing!");
  }

  loadScene(kDefaultScene, gpFramework->getTargetFbo().get());

  server_.setReceiveCallback(TCPMessageType::HmdStateMessage, [&](TCPMessage&& message) {
    HmdState* state = (HmdState*)message.data.data();

    if (scene_)
      scene_->onHmdEvent(*state);
  });

  server_.setReceiveCallback(
      TCPMessageType::ClientResolutionChangeMessage, [&](TCPMessage&& message) {
        onResizeSwapChain(message.header.width, message.header.height);
      });

  server_.setReceiveCallback(TCPMessageType::CameraPoseMessage, [&](TCPMessage&& message) {
    CameraPoseData* data = (CameraPoseData*)message.data.data();

    camera_->setHeadPosition(data->headPos);
    camera_->setUpVector(data->upVec);
    camera_->setHeadTarget(data->headTarget);
  });

  newestLatencyId_.set(0);
  server_.setReceiveCallback(TCPMessageType::LatencyMeasureMessage, [&](TCPMessage&& message) {
    // We simply send the message back.
    newestLatencyId_.set(message.data[0]);
  });
}

void ServerScreenSpaceRenderer::sendMessages() {
  static int id = 0;

  TCPMessage msg;

  Texture::SharedPtr texture = blurredAOFBO_->getColorTexture(0, kEyeLeft);
  if (texture) {
    msg.header.type = TCPMessageType::FullScreenAmbientOcclusionTextureLeft;
    msg.header.id = id++;
    msg.data = gpDevice->getRenderContext()->readTextureSubresource(texture.get(), 0);
    msg.header.size = msg.data.size();
    msg.header.width = texture->getWidth();
    msg.header.height = texture->getHeight();
    server_.send(msg);
  }

  // Only send right eye when server mode is active
  if (stereoServer_) {
    texture = blurredAOFBO_->getColorTexture(0, kEyeRight);
    msg.header.type = TCPMessageType::FullScreenAmbientOcclusionTextureRight;
    msg.header.id = id++;
    msg.data = gpDevice->getRenderContext()->readTextureSubresource(texture.get(), 0);
    msg.header.size = msg.data.size();
    msg.header.width = texture->getWidth();
    msg.header.height = texture->getHeight();
    server_.send(msg);
  }

  TCPMessage latencyMsg;

  latencyMsg.header.type = TCPMessageType::LatencyMeasureMessage;
  latencyMsg.header.id = id;
  latencyMsg.data.resize(1);
  latencyMsg.data[0] = currentLatencyId_;
  latencyMsg.header.size = latencyMsg.data.size();
  server_.send(latencyMsg);
}

void ServerScreenSpaceRenderer::receiveMessages() {}

void ServerScreenSpaceRenderer::setPerFrameVars(const Fbo* targetFbo, EyeType eye) {
  FALCOR_PROFILE("setPerFrameVars");
  auto constantBuffer = rtaoVars_["perFrameConstantBuffer"];
  constantBuffer["invView"] = glm::inverse(camera_->getViewMatrix(eye));
  constantBuffer["viewportDims"] = float2(targetFbo->getWidth(), targetFbo->getHeight());
  float fovY = focalLengthToFovY(camera_->getFocalLength(), Camera::kDefaultFrameHeight);
  constantBuffer["tanHalfFovY"] = tanf(fovY * 0.5f);
  constantBuffer["sampleIndex"] = sampleIndex_++;
  constantBuffer["aoRadius"] = aoRadius_;
  constantBuffer["aoSamples"] = aoSamples_;
  rtaoVars_["gOutput"] = targetFbo->getColorTexture(0, eye);
}

void ServerScreenSpaceRenderer::renderRT(RenderContext* renderContext, const Fbo* targetFbo) {
  FALCOR_PROFILE("renderRT");

  renderContext->clearFbo(targetFbo, kClearColor, 1.0f, 0, FboAttachmentType::All);

  for (EyeType eye : kAllEyes) {

    if (eye == kEyeRight && !stereoServer_)
      break;

    setPerFrameVars(targetFbo, eye);

    scene_->raytrace(
        renderContext,
        rtaoProgram_.get(),
        rtaoVars_,
        uint3(targetFbo->getWidth(), targetFbo->getHeight(), 1),
        eye);
  }
}

void ServerScreenSpaceRenderer::renderAOBlur(
    RenderContext* renderContext,
    const Fbo* inputFbo,
    const Fbo* targetFbo) {
  FALCOR_PROFILE("renderAOBlur");

  renderContext->clearFbo(targetFbo, kClearColor, 1.0f, 0, FboAttachmentType::All);

  for (EyeType eye : kAllEyes) {
    if (eye == kEyeRight && !stereoServer_)
      break;

    auto vars = computePass_->getVars();
    vars["inTex"] = inputFbo->getColorTexture(0, eye);
    vars["outTex"] = targetFbo->getColorTexture(0, eye);

    auto constantBuffer = vars["perFrameConstantBuffer"];

    constantBuffer["kernelSize"] = aoKernel_;

    computePass_->execute(renderContext, {inputFbo->getWidth(), inputFbo->getHeight(), 1});
  }
}

void ServerScreenSpaceRenderer::onFrameRender(
    RenderContext* renderContext,
    const Fbo::SharedPtr& targetFbo) {
  renderContext->clearFbo(targetFbo.get(), kClearColor, 1.0f, 0, FboAttachmentType::All);

  if (scene_) {
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

    receiveMessages();

    // This is the pose we used from the client for rendering, thus also the one for end to end
    // latency.
    currentLatencyId_ = newestLatencyId_.get();

    renderRT(renderContext, rtaoFBO_.get());

    renderAOBlur(renderContext, rtaoFBO_.get(), blurredAOFBO_.get());

    auto constantBuffer = rasterVars["perFrameConstantBuffer"];
    constantBuffer["aoOnly"] = aoOnly_;
    constantBuffer["colorOnly"] = colorOnly_;

    rasterPass_->renderScene(renderContext, targetFbo, [&](EyeType eye) {
      rasterVars["aoTex"] = blurredAOFBO_->getColorTexture(0, eye);
    });
  }

  sendMessages();

  TextRenderer::render(renderContext, gpFramework->getFrameRate().getMsg(), targetFbo, {20, 20});
}

bool ServerScreenSpaceRenderer::onKeyEvent(const KeyboardEvent& keyEvent) {
  if (scene_ && scene_->onKeyEvent(keyEvent))
    return true;
  return false;
}

bool ServerScreenSpaceRenderer::onMouseEvent(const MouseEvent& mouseEvent) {
  return scene_ && scene_->onMouseEvent(mouseEvent);
}

void ServerScreenSpaceRenderer::onHmdEvent(const HmdState& hmdState) {
  if (scene_)
    scene_->onHmdEvent(hmdState);
}

void ServerScreenSpaceRenderer::onResizeSwapChain(uint32_t width, uint32_t height) {
  float h = (float)height;
  float w = (float)width;

  if (camera_) {
    camera_->setFocalLength(18);
    float aspectRatio = (w / h);
    camera_->setAspectRatio(aspectRatio);
  }

  rtaoFBO_ = Fbo::create(width, height);
  blurredAOFBO_ = Fbo::create(width, height);

  for (EyeType eye : kAllEyes) {
    if (eye == kEyeRight && !stereoServer_)
      break;

    texAO_[eye] = Texture::create2D(
        width,
        height,
        ResourceFormat::R32Float,
        1,
        1,
        nullptr,
        Resource::BindFlags::UnorderedAccess | Resource::BindFlags::ShaderResource |
            Resource::BindFlags::RenderTarget);

    texBlurredAO_[eye] = Texture::create2D(
        width,
        height,
        ResourceFormat::R32Float,
        1,
        1,
        nullptr,
        Resource::BindFlags::UnorderedAccess | Resource::BindFlags::ShaderResource |
            Resource::BindFlags::RenderTarget);

    // Attach textures for each eye to FBO
    rtaoFBO_->attachColorTarget(texAO_[eye], 0, 0, 0, Fbo::kAttachEntireMipLevel, eye);
    blurredAOFBO_->attachColorTarget(
        texBlurredAO_[eye], 0, 0, 0, Fbo::kAttachEntireMipLevel, eye);
  }
}

} // namespace split_rendering
