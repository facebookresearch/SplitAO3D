// Copyright (c) Facebook
#pragma once
#include <Falcor.h>

namespace PointCloudPassConstants {
const std::string kProgramFile = "Samples/FalcorServer/PointCloudVisualizationPass.slang";
const std::string kColor = "color";
const std::string kPosition = "position";
const std::string kDepth = "depth";

const std::string kmeshDesc = "meshDesc";
const std::string kViewProj = "viewProj";

const std::string kInstanceOffset = "kInstanceOffset";
const std::string kInstancePointCount = "kInstancePointCount";
} // namespace PointCloudPassConstants

class PointCloudVisualizationPass {
 public:
  struct PointCloudPassData {
    PointCloudPassData(
        const Falcor::Fbo::SharedPtr& fbo,
        const Falcor::Buffer::SharedPtr& points,
        glm::mat4 localToWorld,
        glm::mat4 viewProj,
        const uint32_t instanceOffset,
        const uint32_t instancePointCount)
        : fbo(fbo),
          gpuPoints(points),
          localToWorld(localToWorld),
          viewProj(viewProj),
          instanceOffset(instanceOffset),
          instancePointCount(instancePointCount) {}

    const Falcor::Fbo::SharedPtr& fbo;
    const Falcor::Buffer::SharedPtr& gpuPoints;

    glm::mat4 localToWorld;
    glm::mat4 viewProj;
    uint32_t instanceOffset;
    uint32_t instancePointCount;
  };

  using SharedPtr = std::shared_ptr<PointCloudVisualizationPass>;

  static SharedPtr create(
      Falcor::RenderContext* renderContext = nullptr,
      const Falcor::Dictionary& dict = {});

  // Falcor::RenderPass
  void renderUI(Falcor::Gui::Widgets& widget);

  void run(Falcor::RenderContext* renderContext, const PointCloudPassData& dict);

 private:
  PointCloudVisualizationPass(const Falcor::Dictionary& dict = {});

  void parseDictionary(const Falcor::Dictionary& dict);
  void DrawPoints(Falcor::RenderContext* renderContext, uint32_t numPoints);

  Falcor::Vao::SharedPtr vao_;
  Falcor::GraphicsProgram::SharedPtr program_;
  Falcor::GraphicsVars::SharedPtr programVars_;
  Falcor::GraphicsState::SharedPtr graphicsState_;

  float pointSize_ = 16.0f;
};
