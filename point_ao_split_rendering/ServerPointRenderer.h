// (c) Meta Platforms, Inc. and its affiliates
// Based on
// shared\third-party\Falcor\4.1\Falcor\Source\Samples\HelloDXR\HelloDXR.h

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
#include "argparse.hpp"

#include <fstream>
#include "SSAO.h"

using namespace Falcor;

enum EyeType { kEyeLeft = 0, kEyeRight = 1, kEyeCount = 2 };

const static EyeType kAllEyes[2] = {kEyeLeft, kEyeRight};

namespace split_rendering {

class VertexAnimationSaver {
 public:
  std::vector<PackedStaticVertexData> vertex_data;
  uint32_t number_of_frames = 0;
  uint32_t number_of_changed_frames = 0;

  void save(const char* path) {
    std::ofstream file(path, std::ios::binary);
    file.write((const char*)&number_of_frames, sizeof(uint32_t));
    file.write((const char*)&vertex_data[0], sizeof(PackedStaticVertexData) * vertex_data.size());
  }
};

struct NamedFloatTuple
{
  NamedFloatTuple(std::string name, float val) : name_(name), val_(val) {}
  std::string name_;
  float val_;
};

struct NamedIntTuple
{
  NamedIntTuple(std::string name, int64_t val) : name_(name), val_(val) {}
  std::string name_;
  int64_t val_;
};

struct ProfilingStats
{
  std::vector<NamedFloatTuple> profilingStages_;
  std::vector<NamedIntTuple> networkDataStages_;
};

class ServerPointRenderer : public IRenderer {
 public:
  const Gui::DropdownList kAOTypeDropdown = {
      {AO_TYPE_PER_PIXEL_RTAO, "Per-pixel RTAO"},
      {AO_TYPE_POINT_AO_KD_TREE, "Point RTAO (KDTree)"},
      {AO_TYPE_POINT_AO_HASH, "Point RTAO (Hash)"},
      {AO_TYPE_POINT_AO_HASH_KNN, "Point RTAO (Hash, kNN)"},
      {AO_TYPE_POINT_AO_HASH_UPDATE, "Point RTAO (Hash, Update)"},
      {AO_TYPE_SSAO, "SSAO"}};

  explicit ServerPointRenderer(NetworkServer& server, argparse::ArgumentParser& args)
      : server_(server),
        args_(args),
        aoRadius_(args.get<float>("--aoRadius")),
        exitAfterCameraPath_(args.get<bool>("--exit_after_camera_path")),
        raytracingFramerate_(args.get<int>("--raytracingFramerate")),
        serverFramerate_(args.get<int>("--serverFramerate")),
        aoOnly_(args.get<bool>("--aoOnly")) {

    pointGen_.kNumSamplesPerUnitSquaredEliminated =
        args.get<int>("--numSamplesPerUnitSquaredEliminated");
    pointGen_.kMinSamplesPerInstance =
        args.get<int>("--minSamplesPerInstance");

    std::string selected_renderer = args.get<std::string>("--selected_renderer");

    if (selected_renderer == "RTAO")
      aoType_ = AO_TYPE_PER_PIXEL_RTAO;
    else if (selected_renderer == "PBAO")
      aoType_ = AO_TYPE_POINT_AO_HASH_UPDATE;
    else if (selected_renderer == "SSAO")
      aoType_ = AO_TYPE_SSAO;

    // Create output directories if they don't exist yet

    outputDirectory_ = args.get<std::string>("--output_dir");
    screenshotOutputDirectory_ = outputDirectory_ + "/images/";

    std::filesystem::create_directories(std::filesystem::path{screenshotOutputDirectory_});

    setupNetworkCallbacks();
  }

  void onLoad(RenderContext* renderContext) override;
  void onFrameRender(RenderContext* renderContext, const Fbo::SharedPtr& targetFbo) override;
  void onResizeSwapChain(uint32_t width, uint32_t height) override;
  bool onKeyEvent(const KeyboardEvent& keyEvent) override;
  bool onMouseEvent(const MouseEvent& mouseEvent) override;
  // void onHmdEvent(const HmdState& hmdState) override;
  void onGuiRender(Gui* gui) override;
  void setupNetworkCallbacks();

 private:
  static constexpr const float kMSecToSec = 0.001f;

  RasterScenePass::SharedPtr rasterPass_;
  RasterScenePass::SharedPtr depthNormalsPrepass_;
  Scene::SharedPtr scene_;

  ComputePass::SharedPtr computePass_;
  ComputePass::SharedPtr vertexAnimationComputePass_;

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
  float cosNormalThreshold_ = 0.13f;
  float cosDeltaThreshold_ = 0.7f;
  float updateDeltaCosThreshold = 0.6f;
  float updateDeltaPosFactor = 3.0f;
  float updateDeltaValFactor = 8.0f;
  int32_t raytracingFramerate_ = 30;
  float lastRaytracingTimestamp_ = -100.0f;
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
  Texture::SharedPtr texVisibility_[kEyeCount];
  Texture::SharedPtr texVisibilityDepth_[kEyeCount];
  Fbo::SharedPtr rtaoFBO_;
  Fbo::SharedPtr visibilityFBO_;
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
  int32_t serverFramerate_ = -1;
  float lastServerTimeStamp_ = 0.0f;
  std::unique_ptr<NetworkCompressionBase> networkCompression_;
  uint32_t frameCount_ = 0;
  const float fixedFrameTime = 0.016666666666666f;
  uint32_t exportVertexAnimFrameLimit = 1800;
  std::chrono::steady_clock::time_point lastShadedTime_;
  std::vector<VertexAnimationSaver> vSavers_;
  Falcor::SceneBuilder::InstanceMatrices instanceMatrices_;
  std::vector<bool> meshStaticFlags_;
  bool exportVertexAnims_ = false;
  std::vector<rmcv::mat4x4> cameraPathMatrices_;

  void sendMessages(RenderContext* renderContext);
  void receiveMessages();

  void setPerFrameVars(const Fbo* targetFbo, EyeType eye);
  void renderRT(RenderContext* renderContext, const Fbo* targetFbo);
  void renderAOPoints(RenderContext* renderContext, uint32_t raytracingEnabled);
  void renderAOBlur(RenderContext* renderContext, const Fbo* inputFbo, const Fbo* targetFbo);
  void visualizePoints(RenderContext* renderContext, const Fbo::SharedPtr& targetFbo);
  void loadScene(const std::filesystem::path&, const Fbo* targetFbo);

  void firstFrameInit(RenderContext* renderContext);
  void setupPointStructures(RenderContext* renderContext);
  void setupAutomatedScreenshots();

  void loadCameraPath();

  void initTriangleVisibilityBuffer();
  void saveTriangleVisibilityBuffer();

  void shutdown();


  std::vector<uint32_t> instanceTriangleVisibilityOffsets_;
  std::vector<uint32_t> cpuTriangleVisibilityData_;
  Buffer::SharedPtr triangleVisibilityData_;
  Buffer::SharedPtr triangleVisibilityDataPerFrame_;
  Buffer::SharedPtr triangleVisibilityDataTest_;
  Buffer::SharedPtr triangleVisibilityOffsetData_;

  argparse::ArgumentParser args_;
  bool exitAfterCameraPath_ = false;
  std::string outputDirectory_ = "";
  std::string screenshotOutputDirectory_ = "";
  std::vector<ProfilingStats> profilingStats_;
};

} // namespace split_rendering
