// Copyright (c) Facebook
#include "PointCloudVisualizationPass.h"
using namespace Falcor;

using namespace PointCloudPassConstants;

PointCloudVisualizationPass::SharedPtr PointCloudVisualizationPass::create(
    RenderContext* renderContext,
    const Dictionary& dict) {
  return SharedPtr(new PointCloudVisualizationPass(dict));
}

PointCloudVisualizationPass::PointCloudVisualizationPass(const Dictionary& dict) {
  Program::Desc desc;
  desc.addShaderLibrary(kProgramFile).vsEntry("vsMain").psEntry("psMain");
  program_ = GraphicsProgram::create(desc);
  graphicsState_ = GraphicsState::create();
  graphicsState_->setProgram(program_);
  vao_ = Vao::create(Vao::Topology::TriangleList);
  programVars_ = GraphicsVars::create(program_->getReflector());

  auto depthDesc = DepthStencilState::Desc();

  // This is a debug viz that is run at the end of the pipe, it uses
  // the same depth buffer and color buffer
  depthDesc.setDepthEnabled(true);

  graphicsState_->setDepthStencilState(DepthStencilState::create(depthDesc));
  parseDictionary(dict);
}

void PointCloudVisualizationPass::parseDictionary(const Dictionary& dict) {}

void PointCloudVisualizationPass::DrawPoints(
    Falcor::RenderContext* renderContext,
    uint32_t numPoints) {
  constexpr uint32_t kVerticesInQuad = 6u;
  graphicsState_->setVao(vao_);
  graphicsState_->setProgram(program_);
  renderContext->draw(graphicsState_.get(), programVars_.get(), kVerticesInQuad * numPoints, 0);
}

void PointCloudVisualizationPass::renderUI(Gui::Widgets& widget) {
  widget.slider("Point Size (Viz)", pointSize_, 1.0f, 16.0f);
}

void PointCloudVisualizationPass::run(
    Falcor::RenderContext* renderContext,
    const PointCloudPassData& data) {
  // Don't clear color or depth, we're rendering on top of the existing scene
  graphicsState_->setFbo(data.fbo);

  programVars_->setBuffer("gpuPoints", data.gpuPoints);
  programVars_["perFrameConstantBuffer"]["matViewProj"] = data.viewProj;
  programVars_["perFrameConstantBuffer"]["meshDesc"] = data.localToWorld;
  programVars_["perFrameConstantBuffer"]["invTransposemeshDesc"] =
      glm::transpose(glm::inverse(data.localToWorld));

  programVars_["perFrameConstantBuffer"]["instanceOffset"] = data.instanceOffset;
  programVars_["perFrameConstantBuffer"]["instancePointCount"] = data.instancePointCount;
  programVars_["perFrameConstantBuffer"]["quadSize"] = float{pointSize_ / data.fbo->getWidth()};

  DrawPoints(renderContext, data.instancePointCount);
}