/***************************************************************************
 # Copyright (c) 2015-21, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#pragma once
#include "Falcor.h"
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

  void generateAOMap(RenderContext* renderContext, const Camera* camera, EyeType eye);
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

  Texture::SharedPtr texNormals_[kEyeCount];
  Texture::SharedPtr texDepth_[kEyeCount];
  Texture::SharedPtr texBlurredAO_[kEyeCount];
  Dictionary blurParamDict_;
  ComputePass::SharedPtr computePass_;
  bool applyBlur_ = true;

  Scene::SharedPtr scene_;

  struct {
    FullScreenPass::SharedPtr applySSAOPass;
    Fbo::SharedPtr Fbo;
  } composeData_;
};
