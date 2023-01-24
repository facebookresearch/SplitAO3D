// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

// Based on
// shared\third-party\Falcor\4.1\Falcor\Source\Samples\HelloDXR\HelloDXR.cpp
#include "ServerPointRenderer.h"
#include "BinaryMessageType.h"
#include "nanoflann.hpp"

#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/string_cast.hpp>
//#include <glog/logging.h>
#include <algorithm>
#include <execution>
#include "FLIPScreenshotComparison.h"
#include "LZ4Compression.h"
#include "ZSTDCompression.h"
#include "Utils/Threading.h"
#include "Utils/Math/Matrix.h"
#include "Utils/Math/FalcorMath.h"
#include "Utils/UI/TextRenderer.h"

//using namespace rlr_streaming;

namespace split_rendering {

static const float4 kClearColor(0.38f, 0.52f, 0.10f, 1);
//static const std::string kDefaultScene = "test_scenes/skinned.pyscene";
 //static const std::string kDefaultScene = "test_scenes/bunny.pyscene";
// static const std::string kDefaultScene = "test_scenes/plant.pyscene";
// static const std::string kDefaultScene = "test_scenes/cube.pyscene";
// static const std::string kDefaultScene = "test_scenes/chair.pyscene";
//static const std::string kDefaultScene = "test_scenes/arcade_with_animated_things.pyscene";
// static const std::string kDefaultScene =
//    "test_scenes/arcade_with_animated_things_fewer_objects.pyscene";
// static const std::string kDefaultScene = "Arcade/Arcade.pyscene";
//static const std::string kDefaultScene = "test_scenes/sponza.pyscene";
static const std::string kDefaultScene = "test_scenes/space.pyscene";

void ServerPointRenderer::onGuiRender(Gui* gui) {

  if (noGUI_)
    return;

  Gui::Window w(gui, "Hello DXR Settings", {300, 600}, {10, 30});
  w.dropdown("AO Type", kAOTypeDropdown, aoType_);
  w.text(std::string("Num points changed: ") + std::to_string(frameUpdateInfo_.numChangedPoints));
  w.text(std::string("Num cells alloc'd: ") + std::to_string(frameUpdateInfo_.numAllocatedCells));
  w.text(std::string("Max points changed: ") + std::to_string(maxNumPointsChanged_));
  w.text(std::string("Total points changed: ") + std::to_string(totalNumPointsChanged_));
  w.text(
      std::string("Average points changed: ") +
      std::to_string(totalNumPointsChanged_ / (double)gpFramework->getGlobalClock().getFrame()));

  w.text(std::string("Avg onFrameRender time: ") + std::to_string(1000.0f * smoothedRenderTime_));

  w.checkbox("AO Only", aoOnly_);
  w.checkbox("Point Viz", pointViz_);
  w.checkbox("Color Only", colorOnly_);

  w.checkbox("Raytrace Points", raytraceAOPoints_);
  w.var("Raytracing FPS", raytracingFramerate_, 1, 60);
  w.var("Render Loop FPS", serverFramerate_, 1, 60);
  w.checkbox("Send Messages", sendMessages_);
  w.var("AO samples", aoSamples_, 1, 4096);
  w.var("AO radius", aoRadius_, 0.001f, 5.0f);
  ssao_->setSampleRadius(aoRadius_);
  w.var("Neighbors", numNeighbors_, 1, 16);
  w.var("Int. Radius Mult", interpolationRadiusFactor_, 0.1f, 5.0f);
  w.var("AO filter kernel", aoKernel_, 1, 10);
  if (w.button("Load Scene")) {
    std::filesystem::path path;
    if (openFileDialog(Scene::getFileExtensionFilters(), path)) {
      loadScene(path, gpFramework->getTargetFbo().get());
    }
  }

  w.var("simulated latency (ms)", simulatedLatencyMSec_, -300.0f, 300.0f, 1.0f);
  simulatedLatencySec_ = -simulatedLatencyMSec_ * kMSecToSec;
  w.var("cos(N) threshold", cosNormalThreshold_, 0.0f, 1.0f);
  w.var("cos(delta) threshold", cosDeltaThreshold_, 0.0f, 1.0f);
  w.var("update: cos(N) threshold", updateDeltaCosThreshold, 0.0f, 1.0f);
  w.var("update: delta(pos) threshold", updateDeltaPosFactor, 0.0f, (float)POINT_POS_MAX);
  w.var("update: delta(val) threshold", updateDeltaValFactor, 0.0f, (float)POINT_VAL_MAX);

  if (pointCloudVisualizationPass_)
    pointCloudVisualizationPass_->renderUI(w);
  w.text(std::string("Current time: ") + std::to_string(gpFramework->getGlobalClock().getTime()));

  if (scene_)
    scene_->renderUI(w);
}

void ServerPointRenderer::setupNetworkCallbacks() {

    /*
  server_.setReceiveCallback(TCPMessageType::HmdStateMessage, [&](TCPMessage&& message) {
    HmdState* state = (HmdState*)message.data.data();

    if (scene_)
      scene_->onHmdEvent(*state);
  });

  server_.setReceiveCallback(
      TCPMessageType::ClientResolutionChangeMessage, [&](TCPMessage&& message) {
        onResizeSwapChain(message.header.width, message.header.height);
      });

  server_.setReceiveCallback(TCPMessageType::PAOEndOfInit, [&](TCPMessage&& message) {
    gpFramework->getGlobalClock().setFrame(0);
    gpFramework->getGlobalClock().setTime(0);
    clientInitDone_ = true;
  });

  server_.setReceiveCallback(TCPMessageType::CameraPoseMessage, [&](TCPMessage&& message) {
    CameraPoseData* data = (CameraPoseData*)message.data.data();

    camera_->setHeadPosition(data->headPos);
    camera_->setUpVector(data->upVec);
    camera_->setHeadTarget(data->headTarget);
  });*/
}

void ServerPointRenderer::loadScene(const std::filesystem::path& path, const Fbo* targetFbo) {
  auto sceneBuilder = SceneBuilder::create(path);

  // sort meshes by name

  scene_ = sceneBuilder->getScene();
  if (!scene_)
    return;

  // Init network compression stuff
  // TODO: switch via config file or something
  networkCompression_ = std::make_unique<ZSTDCompression>(15, 4, 0);
  //networkCompression_ = std::make_unique<LZ4Compression>();
  useCompression_ = true;

  camera_ = scene_->getCamera();
  latencyMessage_.header.type = TCPMessageType::NumberOfMessageTypes;

  auto typeConformances = scene_->getTypeConformances();
  auto shaderModules = scene_->getShaderModules();
  // Update the controllers
  float radius = length(scene_->getSceneBounds().extent());
  scene_->setCameraSpeed(radius * 0.25f);
  float nearZ = std::max(0.01f, radius / 75000.0f);
  float farZ = radius * 10;
  camera_->setDepthRange(nearZ, farZ);
  camera_->setAspectRatio((float)targetFbo->getWidth() / (float)targetFbo->getHeight());

  //rasterPass_ = RasterScenePass::create(
  //    scene_, "Samples/FalcorServer/FinalCompositePointAO.ps.slang", "vsMain", "main");
  //rasterPass_->getProgram()->setTypeConformances(typeConformances);
  //rasterPass_->getProgram()->setGenerateDebugInfoEnabled(true);

  // Get scene defines. These need to be set on any program using the scene.
  auto defines = scene_->getSceneDefines();

  // Create raster pass.
  // This utility wraps the creation of the program and vars, and sets the necessary scene defines.
  Program::Desc rasterProgDesc;
  rasterProgDesc.addShaderModules(shaderModules);
  rasterProgDesc.addShaderLibrary("Samples/FalcorServer/FinalCompositePointAO.ps.slang")
      .vsEntry("vsMain")
      .psEntry("main");
  rasterProgDesc.addTypeConformances(typeConformances);

  rasterPass_ = RasterScenePass::create(scene_, rasterProgDesc, defines);
  rasterPass_->getProgram()->setGenerateDebugInfoEnabled(true);

  Program::Desc depthNormalsProgDesc;
  depthNormalsProgDesc.addShaderModules(shaderModules);
  depthNormalsProgDesc.addShaderLibrary("Samples/FalcorServer/DepthNormals.ps.slang")
      .vsEntry("vsMain")
      .psEntry("main");
  depthNormalsProgDesc.addTypeConformances(typeConformances);
  depthNormalsPrepass_ = RasterScenePass::create(scene_, depthNormalsProgDesc, defines);
  depthNormalsPrepass_->getProgram()->setGenerateDebugInfoEnabled(true);

  Sampler::Desc desc;
  desc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear);
  rasterPass_->getVars()["mainSampler"] = Sampler::create(desc);

  computePass_ = ComputePass::create("Samples/FalcorServer/BlurPass.cs.slang");
  computePass_->getProgram()->setGenerateDebugInfoEnabled(true);

  {
    RtProgram::Desc rtProgDesc;
    rtProgDesc.addShaderModules(shaderModules);
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
    auto ao = rtProgDesc.addHitGroup("aoClosestHit", "aoAnyHit");
    sbt->setHitGroup(0, scene_->getGeometryIDs(Scene::GeometryType::TriangleMesh), primary);
    sbt->setHitGroup(1, scene_->getGeometryIDs(Scene::GeometryType::TriangleMesh), ao);

    rtaoProgram_ = RtProgram::create(rtProgDesc, defines);
    rtaoProgram_->setTypeConformances(typeConformances);
    rtaoProgram_->setGenerateDebugInfoEnabled(true);
    rtaoVars_ = RtProgramVars::create(rtaoProgram_, sbt);
  }

  // Create an RT shader for writing into the point data structure -> does not have an output color

  RtProgram::Desc rtPointAOProgDesc;
  rtPointAOProgDesc.addShaderModules(shaderModules);
  rtPointAOProgDesc.addShaderLibrary("Samples/FalcorServer/PointRTAO.rt.slang");

  // 1 for calling TraceRay from RayGen
  rtPointAOProgDesc.setMaxTraceRecursionDepth(1);

  // The largest ray payload struct (PrimaryRayData) is 16 bytes. The payload size should
  // be set as small as possible for maximum performance.
  rtPointAOProgDesc.setMaxPayloadSize(16);

  RtBindingTable::SharedPtr pointAOSbt = RtBindingTable::create(1, 1, scene_->getGeometryCount());
  pointAOSbt->setRayGen(rtPointAOProgDesc.addRayGen("rayGen"));
  pointAOSbt->setMiss(0, rtPointAOProgDesc.addMiss("aoMiss"));
  auto primary_point_ao = rtPointAOProgDesc.addHitGroup("aoClosestHit", "aoAnyHit");
  pointAOSbt->setHitGroup(
      0, scene_->getGeometryIDs(Scene::GeometryType::TriangleMesh), primary_point_ao);

  pointAORaytraceProgram = RtProgram::create(rtPointAOProgDesc, defines);
  pointAORaytraceProgram->setTypeConformances(typeConformances);
  pointAORaytraceProgram->setGenerateDebugInfoEnabled(true);
  pointAOVars_ = RtProgramVars::create(pointAORaytraceProgram, pointAOSbt);

  pointCellAllocStage_.init();
}

void ServerPointRenderer::firstFrameInit(RenderContext* renderContext) {
  // Setup point structures
  setupPointStructures(renderContext);

  pointCellCreateNetworkBufferStage_.init(serverHashGen_);
  pointHashCreateNetworkBufferStage_.init(serverHashGen_);

  if (!sendMessages_)
    return;

  // TODO: refactor to use the same function for this and the other message send function
  const auto send_vector_message = [&](TCPMessageType type, auto& vectorData, int headerId = 0) {
    TCPMessage msg;
    msg.header.type = type;
    msg.header.id = headerId;

    // Resize raw binary buffer
    msg.data.resize(
        vectorData.size() * sizeof(std::remove_reference_t<decltype(vectorData)>::value_type));
    // Copy
    std::memcpy(msg.data.data(), vectorData.data(), msg.data.size());

    msg.header.size = msg.data.size();
    msg.header.width = 0;
    msg.header.height = 0;
    msg.header.timestamp = gpFramework->getGlobalClock().getTime() + simulatedLatencySec_;
    //server_.send(msg);
  };

  send_vector_message(
      TCPMessageType::PAOServerInstanceHashInfo, serverHashGen_.getCPUInstanceHashInfo());

  send_vector_message(
      TCPMessageType::PAOInstancePointInfo, serverHashGen_.getCPUInstancePointInfo());

  send_vector_message(
      TCPMessageType::PAOInstanceToPoissonRadius, pointGen_.getDiskRadiusPerInstance());

  send_vector_message(
      TCPMessageType::PAOCompressedClientAOPoints,
      serverHashGen_.getCPUCompressedClientPointCells());

  send_vector_message(
      TCPMessageType::PAOServerHashToPointCell, serverHashGen_.getCPUCompactHashToPointCell());

  // Send EOF message
  {
    TCPMessage msg;
    msg.header.type = TCPMessageType::PAOEndOfInit;
    msg.header.id = 0;

    msg.header.size = 0;
    msg.header.width = 0;
    msg.header.height = 0;
    //server_.send(msg);
  }
}

void ServerPointRenderer::setupPointStructures(RenderContext* renderContext) {
  pointGen_.generatePointsInScene(scene_, renderContext);
  const auto& numFinalSamplesPerInstance = pointGen_.getNumSamplesPerInstance();
  const auto& sampleOffsetPerInstance = pointGen_.getSampleOffsetPerInstance();
  const auto& diskRadiusPerInstance = pointGen_.getDiskRadiusPerInstance();

  serverHashGen_.generate(scene_, pointGen_);

  kdTreeGen_.generate(
      scene_,
      pointGen_,
      serverHashGen_.getCPUPointCells(),
      serverHashGen_.getCPUInstancePointInfo());
  hashGen_.generate(
      scene_,
      pointGen_,
      serverHashGen_.getCPUPointCells(),
      serverHashGen_.getCPUInstancePointInfo());

  pointCloudVisualizationPass_ = PointCloudVisualizationPass::create(nullptr, {});

  gpuFrameUpdateInfo_ = Buffer::createStructured(
      sizeof(PerFrameUpdateInfo),
      1,
      ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
      Buffer::CpuAccess::None);
}

void ServerPointRenderer::setupAutomatedScreenshots() {
  screenshotHelper_ = {};

  // Camera positions, new camera setups can be exported with the "Minus" key
  automatedCameraSetups_ = {
      [&]() {
        //camera_->setHeadPosition(glm::vec3(-1.682712, 1.612922, 3.208458));
        //camera_->setHeadTarget(glm::vec3(-1.097630, 1.206952, 2.506413));
        camera_->setUpVector(glm::vec3(0.001629, 0.999997, -0.001951));
      }, 
      [&]() {
        //camera_->setHeadPosition(glm::vec3(-1.372512, 0.599973, 0.306184));
        //camera_->setHeadTarget(glm::vec3(-1.355760, 0.419115, -0.677182));
        camera_->setUpVector(glm::vec3(0.000000, 1.000000, 0.000000));
      },
      [&]() {
        //camera_->setHeadPosition(glm::vec3(0.370434, 0.570594, 1.203125));
        //camera_->setHeadTarget(glm::vec3(0.289852, 0.664340, 0.210795));
        camera_->setUpVector(glm::vec3(0.000224, 0.999996, 0.002757));
      },
      [&]() {
        //camera_->setHeadPosition(glm::vec3(-1.259848, 0.272437, -0.148128));
        //camera_->setHeadTarget(glm::vec3(-0.893683, -0.123221, -0.990374));
        camera_->setUpVector(glm::vec3(-0.000676, 0.999999, 0.001559));
      },
      [&]() {
        //camera_->setHeadPosition(glm::vec3(-1.742759, 0.183640, -0.475140));
        //camera_->setHeadTarget(glm::vec3(-0.846588, 0.121357, -0.914456));
        camera_->setUpVector(glm::vec3(-0.001659, 0.999998, 0.000813));
      },
  };

  automatedAOConfigSetups_ = {
      [&]() {
        aoOnly_ = false;
        pointViz_ = false;
      },
      [&]() {
        aoOnly_ = false;
        pointViz_ = true;
      },
      [&]() {
        aoOnly_ = true;
        pointViz_ = false;
      },
  };

  const static std::vector<float> timeStamps = {1.0f, 5.0f};
  const static std::vector<float> latencySetups = {0.0f, 40.0f, 80.0f, 120.0f};

  // Write out color only
  for (uint32_t cameraSetupIndex = 0; cameraSetupIndex < automatedCameraSetups_.size();
       cameraSetupIndex++) {
    for (const auto timeStamp : timeStamps) {
      screenshotHelper_.addFrame(
          [&, cameraSetupIndex, timeStamp]() {
            gpFramework->getGlobalClock().setFrame(0);
            gpFramework->getGlobalClock().setTime(timeStamp);
            automatedCameraSetups_[cameraSetupIndex]();
            aoOnly_ = false;
            colorOnly_ = true;
          },
          [&, cameraSetupIndex, timeStamp]() {
            std::string directoryBase = "automated_screenshots_" +
                std::to_string(pointGen_.getCPUPointData().size()) + "_" +
                std::to_string(timeStamp);
            std::string destinationDirectoryOutput = directoryBase + "/color_only_out";

            std::filesystem::create_directories(destinationDirectoryOutput);

            // Screenshot
            std::string currentFilename = destinationDirectoryOutput + "/" +
                std::to_string(cameraSetupIndex) + "_color_only.png";

            screenshotFBO_->getColorTexture(0)->captureToFile(0, 0, currentFilename);
            colorOnly_ = false;
          });
    }
  }

  // Loop over all camera setups for automated screenshotting
  for (uint32_t cameraSetupIndex = 0; cameraSetupIndex < automatedCameraSetups_.size();
       cameraSetupIndex++) {
    // Loop over all ao configs
    for (uint32_t aoConfigSetupIndex = 0; aoConfigSetupIndex < automatedAOConfigSetups_.size();
         aoConfigSetupIndex++) {
      // Loop over all latencies
      for (const auto latencyMs : latencySetups) {
        // Loop over all timestamps
        for (const auto timeStamp : timeStamps) {
          // Loop over AO types
          for (const auto& aoSetting : kAOTypeDropdown) {
            // We only want to compare point AO (update), SSAO and RTAO
            if (aoSetting.value > AO_TYPE_PER_PIXEL_RTAO &&
                aoSetting.value < AO_TYPE_POINT_AO_HASH_UPDATE)
              continue;

            screenshotHelper_.addFrame(
                [&, cameraSetupIndex, aoConfigSetupIndex, latencyMs, timeStamp]() {
                  logWarning(
                      std::string("Begin camera ") + std::to_string(cameraSetupIndex) + ": " +
                      aoSetting.label);
                  gpFramework->getGlobalClock().setFrame(0);
                  gpFramework->getGlobalClock().setTime(timeStamp);
                  simulatedLatencyMSec_ = latencyMs;
                  simulatedLatencySec_ = -simulatedLatencyMSec_ * kMSecToSec;
                  automatedCameraSetups_[cameraSetupIndex]();
                  automatedAOConfigSetups_[aoConfigSetupIndex]();
                  aoType_ = aoSetting.value;

                  if (aoSetting.value == AO_TYPE_PER_PIXEL_RTAO) {
                    aoSamples_ = 512;
                  } else {
                    aoSamples_ = 4096;
                  }
                },
                [&, cameraSetupIndex, aoConfigSetupIndex, latencyMs, timeStamp]() {
                  logWarning(
                      std::string("End camera ") + std::to_string(cameraSetupIndex) + ": " +
                      aoSetting.label);

                  std::string directoryBase = "automated_screenshots_" +
                      std::to_string(pointGen_.getCPUPointData().size()) + "_" +
                      std::to_string(timeStamp);
                  std::string destinationDirectory = directoryBase + "/flip_color";
                  std::string destinationDirectoryOutput = directoryBase + "/color_out";

                  if (aoOnly_) {
                    destinationDirectoryOutput = directoryBase + "/ao_out";
                    destinationDirectory = directoryBase + "/flip_ao";
                  }

                  if (pointViz_) {
                    destinationDirectoryOutput = directoryBase + "/point_viz_out";
                  }

                  std::filesystem::create_directories(destinationDirectory);

                  std::filesystem::create_directories(destinationDirectoryOutput);

                  // Screenshot
                  std::string currentFilename = destinationDirectoryOutput + "/" +
                      std::to_string(cameraSetupIndex) + "_" + std::string(aoSetting.label) + "_" +
                      std::to_string(aoConfigSetupIndex) + "_" + std::to_string(latencyMs) + ".png";

                  std::string referenceFilename = destinationDirectoryOutput + "/" +
                      std::to_string(cameraSetupIndex) + "_" +
                      std::string(kAOTypeDropdown[AO_TYPE_PER_PIXEL_RTAO].label) + "_" +
                      std::to_string(aoConfigSetupIndex) + "_" + std::to_string(latencySetups[0]) +
                      ".png";

                  screenshotFBO_->getColorTexture(0)->captureToFile(0, 0, currentFilename);

                  // Wait for screenshot to finish, so we can process the images with FLIP
                  // afterwards
                  Threading::finish();

                  if (pointViz_)
                    return;

                  FlipScreenshotComparison::compare(
                      referenceFilename, currentFilename, destinationDirectory);

                  simulatedLatencyMSec_ = 0.0f;
                  simulatedLatencySec_ = -simulatedLatencyMSec_ * kMSecToSec;
                });
          }
        }
      }
    }
  }
}

void ServerPointRenderer::onLoad(RenderContext* renderContext) {
  if (gpDevice->isFeatureSupported(Device::SupportedFeatures::Raytracing) == false) {
    logFatal("Device does not support raytracing!");
  }

  loadScene(kDefaultScene, gpFramework->getTargetFbo().get());

  Dictionary ssao_dict;
  ssao_dict["aoMapSize"] = uint2(512, 512);
  ssao_ = SSAO::create(renderContext, ssao_dict, scene_);
  ssao_->setAOMapSize(uint2(512, 512));

  //server_.startThreads();
}

void ServerPointRenderer::sendMessages(RenderContext* renderContext) {
  FALCOR_PROFILE("sendMessages");
  static int id = 0;

  auto point_cell_update_vec =
      pointCellCreateNetworkBufferStage_.getNetworkCellUpdateInfo(renderContext);

  auto point_hash_update_vec =
      pointHashCreateNetworkBufferStage_.getNetworkHashUpdateInfo(renderContext);

  const auto send_vector_message =
      [&](TCPMessageType type, auto& vectorData, int headerId = 0, bool compressData = false) {
        if (vectorData.empty())
          return;

        TCPMessage msg;
        msg.header.type = type;
        msg.header.id = headerId;

        uint32_t inputNumBytes =
            vectorData.size() * sizeof(std::remove_reference_t<decltype(vectorData)>::value_type);

        msg.header.width = 0;
        msg.header.height = 0;
        msg.header.decompressedSize = inputNumBytes;

        if (compressData) {
          auto& compressedData = msg.data;
          uint32_t numCompressedBytes = 0;

          numCompressedBytes =
              networkCompression_->compressData(vectorData.data(), compressedData, inputNumBytes);

          // Encode the ID of the network compression type in the message width
          msg.header.width = (uint32_t)networkCompression_->getID();

          // Make sure sizes are correct
          msg.header.decompressedSize = inputNumBytes;
          msg.header.size = numCompressedBytes;
        } else {
          msg.data.resize(inputNumBytes);
          std::memcpy(msg.data.data(), vectorData.data(), msg.data.size());
          msg.header.size = msg.data.size();
        }

        msg.header.timestamp = gpFramework->getGlobalClock().getTime() + simulatedLatencySec_;
        //server_.send(msg);
      };

  send_vector_message(
      TCPMessageType::PAOPointCellUpdate, point_cell_update_vec, id++, useCompression_);
  send_vector_message(TCPMessageType::PAOHashUpdate, point_hash_update_vec, id++);

  // We send back the message after we are done rendering.
  if (latencyMessage_.header.type == TCPMessageType::LatencyMeasureMessage) {
    latencyMessage_.header.timestamp += simulatedLatencySec_;
    //server_.send(latencyMessage_);
    latencyMessage_.header.type = TCPMessageType::NumberOfMessageTypes;
  }
}

void ServerPointRenderer::receiveMessages() {
  FALCOR_PROFILE("receiveMessages");
  TCPMessage msg;

  //while (server_.tryPopFront(msg)) {
  //  if (msg.header.type == TCPMessageType::LatencyMeasureMessage) {
  //    latencyMessage_ = msg;
  //  }
  //}
}

void ServerPointRenderer::setPerFrameVars(const Fbo* targetFbo, EyeType eye) {
  FALCOR_PROFILE("setPerFrameVars");
  auto constantBuffer = rtaoVars_["perFrameConstantBuffer"];
  constantBuffer["invView"] = inverse(camera_->getViewMatrix());
  constantBuffer["viewportDims"] = float2(targetFbo->getWidth(), targetFbo->getHeight());
  float fovY = focalLengthToFovY(camera_->getFocalLength(), Camera::kDefaultFrameHeight);
  constantBuffer["tanHalfFovY"] = tanf(fovY * 0.5f);
  constantBuffer["sampleIndex"] = sampleIndex_++;
  constantBuffer["aoRadius"] = aoRadius_;
  constantBuffer["aoSamples"] = aoSamples_;
  rtaoVars_["gOutput"] = targetFbo->getColorTexture(0);
}

void ServerPointRenderer::renderRT(RenderContext* renderContext, const Fbo* targetFbo) {
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
        uint3(targetFbo->getWidth(), targetFbo->getHeight(), 1));
  }
}

void ServerPointRenderer::renderAOPoints(RenderContext* renderContext) {
  FALCOR_PROFILE("renderAOPoints");

  uint32_t raytracingEnabled = 0;
  float currentTime = gpFramework->getGlobalClock().getTime();

  if ((currentTime - lastRaytracingTimestamp_) >= (1.0f / raytracingFramerate_)) {
    raytracingEnabled = 1;
    lastRaytracingTimestamp_ = currentTime;
  }

  // If the screenshotting automation is running, we always raytrace
  if (screenshotHelper_.isRunning()) {
    raytracingEnabled = 1;
  }

  auto constantBuffer = pointAOVars_["perFrameConstantBuffer"];
  constantBuffer["sampleIndex"] = sampleIndex_++;
  constantBuffer["aoRadius"] = aoRadius_;
  constantBuffer["aoSamples"] = aoSamples_;
  constantBuffer["raytracePoints"] = raytracingEnabled;
  constantBuffer["updateDeltaCosThreshold"] = updateDeltaCosThreshold;
  constantBuffer["updateDeltaPosFactor"] = updateDeltaPosFactor;
  constantBuffer["updateDeltaValFactor"] = updateDeltaValFactor;

  pointAOVars_["frameUpdateInfo"] = gpuFrameUpdateInfo_;
  pointAOVars_["serverAOPoints"] = serverHashGen_.getGPUPointCells();
  pointAOVars_["compressedClientAOPoints"] = serverHashGen_.getGPUCompressedClientPointCells();
  pointAOVars_["instancePointInfo"] = serverHashGen_.getGPUInstancePointInfo();
  pointAOVars_["instanceToDiskRadius"] = pointGen_.getGPUDiskRadiusPerInstance();
  pointAOVars_["serverInstanceHashInfo"] = serverHashGen_.getGPUInstanceHashInfo();
  pointAOVars_["pointUpdateData"] = serverHashGen_.getGPUPointUpdateData();
  pointAOVars_["cellAllocIndirectDispatchArgs"] = pointCellAllocStage_.getIndirectBuffer();
  pointAOVars_["cellNetworkBufferIndirectDispatchArgs"] =
      pointCellCreateNetworkBufferStage_.getCellIndirectBuffer();
  pointAOVars_["hashNetworkBufferIndirectDispatchArgs"] =
      pointHashCreateNetworkBufferStage_.getHashIndirectBuffer();
  pointAOVars_["serverHashToPointCell"] = serverHashGen_.getGPUHashToPointCell();
  pointAOVars_["hashNumBuckets"] = serverHashGen_.getGPUHashNumBuckets();
  pointAOVars_["cellDirtyFlags"] = serverHashGen_.getGPUCellDirtyFlags();
  pointAOVars_["cellDirtyInfos"] = pointCellCreateNetworkBufferStage_.getDirtyCellInfoBuffer();
  pointAOVars_["hashDirtyInfos"] = pointHashCreateNetworkBufferStage_.getDirtyHashInfoBuffer();

  scene_->raytrace(
      renderContext,
      pointAORaytraceProgram.get(),
      pointAOVars_,
      uint3(serverHashGen_.getGPUPointCells()->getElementCount(), 1, 1));

  frameUpdateInfo_ = *(PerFrameUpdateInfo*)gpuFrameUpdateInfo_->map(Buffer::MapType::Read);

  if (gpFramework->getGlobalClock().getFrame() > 100) {
    minNumPointsChanged_ = std::min(frameUpdateInfo_.numChangedPoints, minNumPointsChanged_);
    maxNumPointsChanged_ = std::max(frameUpdateInfo_.numChangedPoints, maxNumPointsChanged_);
  }
  totalNumPointsChanged_ += frameUpdateInfo_.numChangedPoints;
  gpuFrameUpdateInfo_->unmap();
}

void ServerPointRenderer::renderAOBlur(
    RenderContext* renderContext,
    const Fbo* inputFbo,
    const Fbo* targetFbo) {
  FALCOR_PROFILE("renderAOBlur");

  renderContext->clearFbo(targetFbo, kClearColor, 1.0f, 0, FboAttachmentType::All);

  for (EyeType eye : kAllEyes) {
    if (eye == kEyeRight && !stereoServer_)
      break;

    auto vars = computePass_->getVars();
    vars["inTex"] = inputFbo->getColorTexture(0);
    vars["outTex"] = targetFbo->getColorTexture(0);

    auto constantBuffer = vars["perFrameConstantBuffer"];

    constantBuffer["kernelSize"] = aoKernel_;

    computePass_->execute(renderContext, {inputFbo->getWidth(), inputFbo->getHeight(), 1});
  }
}
void ServerPointRenderer::visualizePoints(
    RenderContext* renderContext,
    const Fbo::SharedPtr& targetFbo) {
  FALCOR_PROFILE("visualizePoints");
  const auto& animationController = scene_->getAnimationController();
  const auto& globalTransforms = animationController->getGlobalMatrices();
  const auto& invTransposeGlobalTransforms = animationController->getInvTransposeGlobalMatrices();
  uint32_t meshCount = scene_->getMeshCount();

  const auto instanceCount = scene_->getGeometryInstanceCount();

  for (uint32_t instanceId = 0; instanceId < instanceCount; instanceId++) {
    const auto& instance = scene_->getGeometryInstance(instanceId);

    const auto& localToWorld = globalTransforms[instance.globalMatrixID];
    const auto& invTranspLocalToWorld = invTransposeGlobalTransforms[instance.globalMatrixID];

    PointCloudVisualizationPass::PointCloudPassData data(
        targetFbo,
        serverHashGen_.getGPUPointCells(),
        localToWorld,
        invTranspLocalToWorld,
        camera_->getViewProjMatrix(),
        serverHashGen_.getCPUInstancePointInfo()[instanceId].pointCellOffset,
        serverHashGen_.getCPUInstancePointInfo()[instanceId].maxNumPoints);

    pointCloudVisualizationPass_->run(renderContext, data);
  }
}

void ServerPointRenderer::onFrameRender(
    RenderContext* renderContext,
    const Fbo::SharedPtr& targetFbo) {
  if (!firstFrameInitDone_) {
    firstFrameInit(renderContext);
    firstFrameInitDone_ = true;
  }

  if (sendMessages_ && !clientInitDone_)
    return;

  receiveMessages();

  screenshotHelper_.beginFrame();

  // We only run the framerate limiting code if we are not running in the "automated screenshotting"
  // mode
  if (!screenshotHelper_.isRunning()) {
    if ((gpFramework->getGlobalClock().getTime() - lastServerTimeStamp_) <
        (1.0f / serverFramerate_)) {
      return;
    }
  }

  lastServerTimeStamp_ = gpFramework->getGlobalClock().getTime();

  auto start = std::chrono::high_resolution_clock::now();
  renderContext->clearFbo(screenshotFBO_.get(), kClearColor, 1.0f, 0, FboAttachmentType::All);

  if (scene_) {
    double sceneTime = gpFramework->getGlobalClock().getTime();

    scene_->update(renderContext, sceneTime + simulatedLatencySec_);

    if (raytraceAOPoints_) {
      // only update the first portion that tracks the number of changed points per frame
      PerFrameUpdateInfo zero = {0, 0};
      renderContext->updateBuffer(gpuFrameUpdateInfo_.get(), &zero, 0, 4);
      renderAOPoints(renderContext);
      pointCellAllocStage_.execute(renderContext, serverHashGen_, pointGen_);
      //pointCellCreateNetworkBufferStage_.execute(renderContext, serverHashGen_, pointGen_);
      //pointHashCreateNetworkBufferStage_.execute(renderContext, serverHashGen_, pointGen_);
    }
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

    // This is the pose we used from the client for rendering, thus also the one for end to end
    // latency.
    if (aoType_ == AO_TYPE_PER_PIXEL_RTAO) {
      renderRT(renderContext, rtaoFBO_.get());
      renderAOBlur(renderContext, rtaoFBO_.get(), blurredAOFBO_.get());
    }

    if (false && aoType_ == AO_TYPE_SSAO) {
      // Render SSAO baseline
      for (EyeType eye : kAllEyes) {
        if (!targetFbo->getColorTexture(0))
          break;

        {
          FALCOR_PROFILE("SSAO");
          ssao_->generateAOMap(renderContext, camera_.get(), eye);
        }
      }
    }

    // TODO/Hack: this is essentially the same hack as on the client side - we use the previous
    // frame's transformations to store the transformations with the simulated latency. This is a
    // bit wasteful, but should not be too bad.
    scene_->update(renderContext, sceneTime);

    auto constantBuffer = rasterVars["perFrameConstantBuffer"];
    constantBuffer["aoOnly"] = aoOnly_;
    constantBuffer["colorOnly"] = colorOnly_;
    constantBuffer["aoType"] = aoType_;
    constantBuffer["aoRadius"] = aoRadius_;
    constantBuffer["interpolationRadiusFactor"] = interpolationRadiusFactor_;
    constantBuffer["cosNormalThreshold"] = cosNormalThreshold_;
    constantBuffer["cosDeltaThreshold"] = cosDeltaThreshold_;
    rasterVars["serverAOPoints"] = serverHashGen_.getGPUPointCells();
    rasterVars["compressedClientAOPoints"] = serverHashGen_.getGPUCompressedClientPointCells();
    rasterVars["instanceToDiskRadius"] = pointGen_.getGPUDiskRadiusPerInstance();
    rasterVars["linearTree"] = kdTreeGen_.getGPUKDTree();
    rasterVars["kdTreeIndex"] = kdTreeGen_.getGPUKDTreeIndex();
    rasterVars["instanceToKdTree"] = kdTreeGen_.getGPUInstanceKDTreeOffset();
    rasterVars["instanceToPointOffset"] = kdTreeGen_.getGPUInstanceKDTreeIndexOffset();
    rasterVars["hashToBucket"] = hashGen_.getGPUHashToBucket();
    rasterVars["hashBucketToPointCell"] = hashGen_.getGPUHashBucketToPointCell();
    rasterVars["pointCells"] = hashGen_.getGPUPointCells();
    rasterVars["instanceHashInfo"] = hashGen_.getGPUInstanceHashInfo();
    rasterVars["serverInstanceHashInfo"] = serverHashGen_.getGPUInstanceHashInfo();
    rasterVars["instancePointInfo"] = serverHashGen_.getGPUInstancePointInfo();
    rasterVars["serverHashToPointCell"] = serverHashGen_.getGPUHashToPointCell();
    rasterVars["hashNumBuckets"] = serverHashGen_.getGPUHashNumBuckets();
    auto kdTreeConstantBuffer = rasterVars["kdTreeConstantBuffer"];
    kdTreeConstantBuffer["numNeighbors"] = numNeighbors_;

    /*
    rasterPass_->renderScene(renderContext, screenshotFBO_, [&](EyeType eye) {
      rasterVars["aoTex"] = blurredAOFBO_->getColorTexture(0, eye);
      rasterVars["ssaoTex"] = ssao_->getBlurredAOFBO()->getColorTexture(0, eye);
    });*/

    rasterVars["aoTex"] = blurredAOFBO_->getColorTexture(0);
    rasterVars["ssaoTex"] = ssao_->getBlurredAOFBO()->getColorTexture(0);
    rasterPass_->renderScene(renderContext, screenshotFBO_);

    if (pointViz_) {
      visualizePoints(renderContext, screenshotFBO_);
    }

    renderContext->blit(
        screenshotFBO_->getColorTexture(0)->getSRV(), targetFbo->getRenderTargetView(0));
  }

  if (sendMessages_)
    sendMessages(renderContext);

  if (!noGUI_)
    TextRenderer::render(renderContext, gpFramework->getFrameRate().getMsg(), targetFbo, {20, 20});

  auto end = std::chrono::high_resolution_clock::now();

  auto duration = end - start;
  auto duration_sec = std::chrono::duration_cast<std::chrono::duration<float>>(duration).count();

  smoothedRenderTime_ =
      duration_sec * movingAverageFactor_ + smoothedRenderTime_ * (1.0f - movingAverageFactor_);

  screenshotHelper_.endFrame();
}

bool ServerPointRenderer::onKeyEvent(const KeyboardEvent& keyEvent) {
  if (keyEvent.type == KeyboardEvent::Type::KeyReleased) {

      /*
    if (keyEvent.key == KeyboardEvent::Key::Minus) {
      // Export current cam position / orientation

      std::string cameraDumpStr = "";

      const auto& headPos = camera_->getHeadPosition();
      const auto& headTarget = camera_->getHeadTarget();
      const auto& upVec = camera_->getUpVector();

      cameraDumpStr += "camera_->setHeadPosition(glm::" + glm::to_string(headPos) + ");\n";
      cameraDumpStr += "camera_->setHeadTarget(glm::" + glm::to_string(headTarget) + ");\n";
      cameraDumpStr += "camera_->setUpVector(glm::" + glm::to_string(upVec) + ");\n";

      logWarning("Camera dump: ");
      logWarning(cameraDumpStr);

      screenshotFBO_->getColorTexture(0)->captureToFile(0, 0, "texture.png");
    } else */

    if (keyEvent.key == Input::Key::Equal) {
      // Setup screenshot helper
      setupAutomatedScreenshots();
    } else if (keyEvent.key == Input::Key::RightBracket) {
      // Reset time stamp
      gpFramework->getGlobalClock().setFrame(0);
      gpFramework->getGlobalClock().setTime(0);
      lastServerTimeStamp_ = 0.0f;
      lastRaytracingTimestamp_ = 0.0f;

      if (sendMessages_) {
        TCPMessage msg;
        msg.header.type = TCPMessageType::PAOEndOfInit;
        msg.header.id = 0;

        msg.header.size = 0;
        msg.header.width = 0;
        msg.header.height = 0;
        //server_.send(msg);
      }
    } else if (keyEvent.key == Input::Key::N) {
      noGUI_ = !noGUI_;
    }
  }

  if (scene_ && scene_->onKeyEvent(keyEvent))
    return true;
  return false;
}

bool ServerPointRenderer::onMouseEvent(const MouseEvent& mouseEvent) {
  return scene_ && scene_->onMouseEvent(mouseEvent);
}

/*
void ServerPointRenderer::onHmdEvent(const HmdState& hmdState) {
  if (scene_)
    scene_->onHmdEvent(hmdState);
}*/

void ServerPointRenderer::onResizeSwapChain(uint32_t width, uint32_t height) {
  float h = (float)height;
  float w = (float)width;

  ssao_->setAOMapSize(uint2(width, height));

  if (camera_) {
    camera_->setFocalLength(18);
    float aspectRatio = (w / h);
    camera_->setAspectRatio(aspectRatio);
  }

  rtaoFBO_ = Fbo::create();
  screenshotFBO_ = Fbo::create();
  blurredAOFBO_ = Fbo::create();

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

    texScreenshot_[eye] = Texture::create2D(
        width,
        height,
        ResourceFormat::BGRA8UnormSrgb,
        1,
        1,
        nullptr,
        Resource::BindFlags::ShaderResource | Resource::BindFlags::RenderTarget);

    texDepth_[eye] = Texture::create2D(
        width,
        height,
        ResourceFormat::D32Float,
        1,
        1,
        nullptr,
        Resource::BindFlags::ShaderResource | Resource::BindFlags::DepthStencil);

    // Attach textures for each eye to FBO
    rtaoFBO_->attachColorTarget(texAO_[eye], 0, 0, 0, Fbo::kAttachEntireMipLevel);
    blurredAOFBO_->attachColorTarget(texBlurredAO_[eye], 0, 0, 0, Fbo::kAttachEntireMipLevel);
    screenshotFBO_->attachColorTarget(texScreenshot_[eye], 0);
    screenshotFBO_->attachDepthStencilTarget(texDepth_[eye], 0, 0, Fbo::kAttachEntireMipLevel);
  }
}

} // namespace split_rendering
