// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

// Based on
// shared\third-party\Falcor\4.1\Falcor\Source\Samples\HelloDXR\HelloDXR.h

#pragma once

#include "NetworkServer.h"
#include "PointCellAllocationStage.h"
#include "PointCellCreateNetworkBufferStage.h"
#include "PointCloudVisualizationPass.h"
#include "PointHashCreateNetworkBufferStage.h"

#include <Falcor.h>
#include "PointData.slang"
#include "Rendering/Lights/EnvMapLighting.h"
#include "ScreenshotCaptureHelper.h"

#include <atomic>
#include "MeshPointGenerator.h"
#include "NetworkCompressionBase.h"
#include "PointAOConstantsShared.slangh"
#include "PointHashGenerator.h"
#include "PointKDTreeGenerator.h"
#include "PointServerHashGenerator.h"
#include "RenderGraph/BasePasses/RasterScenePass.h"

#include "SSAO.h"

using namespace Falcor;

enum EyeType
{
    kEyeLeft = 0,
    kEyeRight = 1,
    kEyeCount = 2
};

const static EyeType kAllEyes[2] = {kEyeLeft, kEyeRight};

namespace split_rendering {

class ServerPointRenderer : public IRenderer {
 public:
  const Gui::DropdownList kAOTypeDropdown = {
      {AO_TYPE_PER_PIXEL_RTAO, "Per-pixel RTAO"},
      {AO_TYPE_POINT_AO_KD_TREE, "Point RTAO (KDTree)"},
      {AO_TYPE_POINT_AO_HASH, "Point RTAO (Hash)"},
      {AO_TYPE_POINT_AO_HASH_KNN, "Point RTAO (Hash, kNN)"},
      {AO_TYPE_POINT_AO_HASH_UPDATE, "Point RTAO (Hash, Update)"},
      {AO_TYPE_SSAO, "SSAO"}};

  explicit ServerPointRenderer(NetworkServer& server) : server_(server) {
    setupNetworkCallbacks();
  }

  void onLoad(RenderContext* renderContext) override;
  void onFrameRender(RenderContext* renderContext, const Fbo::SharedPtr& targetFbo) override;
  void onResizeSwapChain(uint32_t width, uint32_t height) override;
  bool onKeyEvent(const KeyboardEvent& keyEvent) override;
  bool onMouseEvent(const MouseEvent& mouseEvent) override;
  //void onHmdEvent(const HmdState& hmdState) override;
  void onGuiRender(Gui* gui) override;
  void setupNetworkCallbacks();

 private:
  static constexpr const float kMSecToSec = 0.001f;

  RasterScenePass::SharedPtr rasterPass_;
  RasterScenePass::SharedPtr depthNormalsPrepass_;
  Scene::SharedPtr scene_;

  ComputePass::SharedPtr computePass_;

  RtProgram::SharedPtr rtaoProgram_ = nullptr;
  RtProgram::SharedPtr pointAORaytraceProgram = nullptr;
  Camera::SharedPtr camera_;
  SSAO::SharedPtr ssao_;

  bool aoOnly_ = false;
  bool colorOnly_ = false;
  bool pointViz_ = false;
  bool raytraceAOPoints_ = true;
  bool sendMessages_ = false;
  bool noGUI_ = false;
  bool useCompression_ = false;
  float simulatedLatencySec_ = 0.0f;
  float simulatedLatencyMSec_ = 0.0f;
  uint32_t aoType_ = AO_TYPE_POINT_AO_HASH_UPDATE;

  // This flag enables rendering of the AO contribution in stereo, but does not open a stereo
  // window. This needs to be set, otherwise stereo rendering is completely disabled. To view the
  // server output in stereo, enable the stereo flag in the falcor config settings in
  // ServerMain.cpp
  bool stereoServer_ = false;
  int aoKernel_ = 3;
  int aoSamples_ = 192;
  float aoRadius_ = 0.6f;
  float interpolationRadiusFactor_ = 1.0f;
  int numNeighbors_ = 8;
  float cosNormalThreshold_ = 0.2f;
  float cosDeltaThreshold_ = 0.1f;
  float updateDeltaCosThreshold = 0.9f;
  float updateDeltaPosFactor = 1.0f;
  float updateDeltaValFactor = 6.0f;
  int32_t raytracingFramerate_ = 30;
  float lastRaytracingTimestamp_ = 0.0f;
  PerFrameUpdateInfo frameUpdateInfo_{0, 0};
  uint32_t minNumPointsChanged_ = std::numeric_limits<uint32_t>::max();
  uint32_t maxNumPointsChanged_ = 0;
  uint64_t totalNumPointsChanged_ = 0;
  bool firstFrameInitDone_ = false;
  bool clientInitDone_ = false;

  RtProgramVars::SharedPtr rtaoVars_;
  RtProgramVars::SharedPtr pointAOVars_;
  Texture::SharedPtr texAO_[kEyeCount];
  Texture::SharedPtr texBlurredAO_[kEyeCount];
  Fbo::SharedPtr rtaoFBO_;
  Fbo::SharedPtr blurredAOFBO_;
  Fbo::SharedPtr screenshotFBO_;
  Texture::SharedPtr texScreenshot_[kEyeCount];
  Texture::SharedPtr texDepth_[kEyeCount];
  GraphicsVars::SharedPtr perEyeVars_;

  uint32_t sampleIndex_ = 0xdeadbeef;

  NetworkServer& server_;

  std::atomic<uint8_t> newestLatencyId_;
  uint8_t currentLatencyId_;
  EnvMapLighting::SharedPtr envMapLighting_;

  MeshPointGenerator pointGen_;
  PointKDTreeGenerator kdTreeGen_;
  PointHashGenerator hashGen_;
  PointServerHashGenerator serverHashGen_;
  TCPMessage latencyMessage_;

  Buffer::SharedPtr gpuFrameUpdateInfo_;
  Buffer::SharedPtr gpuNumChangedCells;

  PointCloudVisualizationPass::SharedPtr pointCloudVisualizationPass_;
  PointCellAllocationStage pointCellAllocStage_;
  PointCellCreateNetworkBufferStage pointCellCreateNetworkBufferStage_;
  PointHashCreateNetworkBufferStage pointHashCreateNetworkBufferStage_;
  ScreenshotCaptureHelper screenshotHelper_;
  std::vector<std::function<void()>> automatedCameraSetups_;
  std::vector<std::function<void()>> automatedAOConfigSetups_;

  static constexpr float movingAverageFactor_ = 0.3f;
  float smoothedRenderTime_ = 0.0f;
  int32_t serverFramerate_ = 30;
  float lastServerTimeStamp_ = 0.0f;
  std::unique_ptr<NetworkCompressionBase> networkCompression_;

  void sendMessages(RenderContext* renderContext);
  void receiveMessages();

  void setPerFrameVars(const Fbo* targetFbo, EyeType eye);
  void renderRT(RenderContext* renderContext, const Fbo* targetFbo);
  void renderAOPoints(RenderContext* renderContext);
  void renderAOBlur(RenderContext* renderContext, const Fbo* inputFbo, const Fbo* targetFbo);
  void visualizePoints(RenderContext* renderContext, const Fbo::SharedPtr& targetFbo);
  void loadScene(const std::filesystem::path&, const Fbo* targetFbo);

  void firstFrameInit(RenderContext* renderContext);
  void setupPointStructures(RenderContext* renderContext);
  void setupAutomatedScreenshots();
};

} // namespace split_rendering
