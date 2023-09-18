/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */


#pragma once
#include "Falcor.h"
#include "RenderGraph/BasePasses/FullScreenPass.h"
#include "RenderGraph/BasePasses/RasterScenePass.h"
#include "RenderGraph/RenderGraph.h"
#include "SSAOData.slang"

using namespace Falcor;

class SSAO : public RenderPass {
 public:
  using SharedPtr = std::shared_ptr<SSAO>;

  static const Info kInfo;

  enum class SampleDistribution : uint32_t { Random, UniformHammersley, CosineHammersley };

  SSAO(const Scene::SharedPtr& scene);
  static SharedPtr
  create(RenderContext* renderContext, const Dictionary& dict, const Scene::SharedPtr& scene);

  virtual Dictionary getScriptingDictionary() override;
  virtual RenderPassReflection reflect(const CompileData& compileData) override;
  virtual void compile(RenderContext* renderContext, const CompileData& compileData) override;
  virtual void execute(RenderContext* renderContext, const RenderData& renderData) override{};
  virtual void setScene(RenderContext* renderContext, const Scene::SharedPtr& scene) override {
    scene_ = scene;
  }
  virtual void renderUI(Gui::Widgets& widget) override;
  void setAOMapSize(uint2 size) {
    aoMapSize_ = size;
    init();
  }

  void renderAOBlur(
      RenderContext* renderContext,
      Texture::SharedPtr& inputTex,
      Texture::SharedPtr& outputTex);
  void renderDepthNormalsPrepass(RenderContext* renderContext, const Fbo::SharedPtr& targetFbo);

  void setSampleRadius(float radius);
  void setKernelSize(uint32_t kernelSize);
  void setDistribution(uint32_t distribution);
  float getSampleRadius() {
    return ssaoData_.radius;
  }
  uint32_t getKernelSize() {
    return ssaoData_.kernelSize;
  }
  uint32_t getDistribution() {
    return (uint32_t)hemisphereDistribution_;
  }
  Fbo::SharedPtr& getBlurredAOFBO() {
    return blurredSSAOFbo_;
  }

  void init();

  void generateAOMap(RenderContext* renderContext, const Camera* camera, uint32_t eye);
  void setNoiseTexture(uint32_t width, uint32_t height);

 private:
  void setKernel();

  SSAOData ssaoData_;
  bool isDirty_ = false;

  Fbo::SharedPtr aoFBO_;
  uint2 aoMapSize_ = uint2(1024);
  RasterScenePass::SharedPtr depthNormalsPrepass_;

  Sampler::SharedPtr noiseSampler_;
  Texture::SharedPtr noiseTexture_;
  uint2 noiseSize_ = uint2(16);

  Sampler::SharedPtr textureSampler_;
  SampleDistribution hemisphereDistribution_ = SampleDistribution::CosineHammersley;

  FullScreenPass::SharedPtr ssaoPass_;
  RenderGraph::SharedPtr blurGraph_;
  Fbo::SharedPtr depthNormalsFbo_;
  Fbo::SharedPtr blurredSSAOFbo_;

  Texture::SharedPtr texNormals_[2];
  Texture::SharedPtr texDepth_[2];
  Texture::SharedPtr texBlurredAO_[2];
  Dictionary blurParamDict_;
  ComputePass::SharedPtr computePass_;
  bool applyBlur_ = true;

  Scene::SharedPtr scene_;

  struct {
    FullScreenPass::SharedPtr applySSAOPass;
    Fbo::SharedPtr Fbo;
  } composeData_;
};
