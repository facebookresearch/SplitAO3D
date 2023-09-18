/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#pragma once
#include <Falcor.h>
#include "Utils/Math/FalcorMath.h"
#include "Utils/Math/Matrix.h"
#include "PointServerHashGenerator.h"

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
        const split_rendering::PointServerHashGenerator& hash_gen,
        Falcor::float4x4 localToWorld,
        Falcor::float4x4 invTranspLocalToWorld,
        Falcor::float4x4 viewProj,
        const uint32_t instanceOffset,
        const uint32_t instancePointCount)
        : fbo(fbo),
          hashGen(hash_gen),
          localToWorld(localToWorld),
          invTranspLocalToWorld(invTranspLocalToWorld),
          viewProj(viewProj),
          instanceOffset(instanceOffset),
          instancePointCount(instancePointCount) {}

    const Falcor::Fbo::SharedPtr& fbo;
    const split_rendering::PointServerHashGenerator& hashGen;

    Falcor::float4x4 localToWorld;
    Falcor::float4x4 invTranspLocalToWorld;
    Falcor::float4x4 viewProj;
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
