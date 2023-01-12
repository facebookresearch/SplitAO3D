/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
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
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
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
#include "SSAO.h"
#include "glm/gtc/random.hpp"
#include "Utils/Math/FalcorMath.h"

using namespace Falcor;

const RenderPass::Info SSAO::kInfo{
    "SSAO",
    "Screen-space ambient occlusion. Can be used with and without a normal-map."};

namespace {
const Gui::DropdownList kDistributionDropdown = {
    {(uint32_t)SSAO::SampleDistribution::Random, "Random"},
    {(uint32_t)SSAO::SampleDistribution::UniformHammersley, "Uniform Hammersley"},
    {(uint32_t)SSAO::SampleDistribution::CosineHammersley, "Cosine Hammersley"}};

const std::string kAoMapSize = "aoMapSize";
const std::string kKernelSize = "kernelSize";
const std::string kNoiseSize = "noiseSize";
const std::string kDistribution = "distribution";
const std::string kRadius = "radius";
const std::string kBlurKernelWidth = "blurWidth";
const std::string kBlurSigma = "blurSigma";

const std::string kColorIn = "colorIn";
const std::string kColorOut = "colorOut";
const std::string kDepth = "depth";
const std::string kNormals = "normals";
const std::string kAoMap = "AoMap";

const std::string kSSAOShader = "RenderPasses/SSAO/SSAO.ps.slang";
const std::string kApplySSAOShader = "RenderPasses/SSAO/ApplyAO.ps.slang";
} // namespace

SSAO::SSAO(const Scene::SharedPtr& scene) : RenderPass(kInfo), scene_(scene) {
  Sampler::Desc samplerDesc;
  samplerDesc.setFilterMode(Sampler::Filter::Point, Sampler::Filter::Point, Sampler::Filter::Point)
      .setAddressingMode(
          Sampler::AddressMode::Wrap, Sampler::AddressMode::Wrap, Sampler::AddressMode::Wrap);
  noiseSampler_ = Sampler::create(samplerDesc);

  samplerDesc
      .setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear)
      .setAddressingMode(
          Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp);
  textureSampler_ = Sampler::create(samplerDesc);

  ssaoPass_ = FullScreenPass::create(kSSAOShader);
  composeData_.applySSAOPass = FullScreenPass::create(kApplySSAOShader);
  Sampler::Desc desc;
  desc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear);
  composeData_.applySSAOPass["gSampler"] = Sampler::create(desc);
  composeData_.Fbo = Fbo::create();

  auto typeConformances = scene_->getTypeConformances();
  depthNormalsPrepass_ =
      RasterScenePass::create(scene_, "Samples/FalcorServer/DepthNormals.ps.slang", "vsMain", "main");
  depthNormalsPrepass_->getProgram()->setTypeConformances(typeConformances);

  computePass_ = ComputePass::create("Samples/FalcorServer/BlurPass.cs.slang");
  computePass_->getProgram()->setTypeConformances(typeConformances);
}

SSAO::SharedPtr
SSAO::create(RenderContext* renderContext, const Dictionary& dict, const Scene::SharedPtr& scene) {
  SharedPtr ssao = std::make_shared<SSAO>(scene);
  Dictionary blurDict;
  for (const auto& [key, value] : dict) {
    if (key == kAoMapSize)
      ssao->aoMapSize_ = value;
    else if (key == kKernelSize)
      ssao->ssaoData_.kernelSize = value;
    else if (key == kNoiseSize)
      ssao->noiseSize_ = value;
    else if (key == kDistribution)
      ssao->hemisphereDistribution_ = value;
    else if (key == kRadius)
      ssao->ssaoData_.radius = value;
    else if (key == kBlurKernelWidth)
      ssao->blurParamDict_["kernelWidth"] = (uint32_t)value;
    else if (key == kBlurSigma)
      ssao->blurParamDict_["sigma"] = (float)value;
    else
      logWarning("Unknown field '{}' in a SSAO dictionary.", key);
  }
  return ssao;
}

Dictionary SSAO::getScriptingDictionary() {
  Dictionary dict;
  dict[kAoMapSize] = aoMapSize_;
  dict[kKernelSize] = ssaoData_.kernelSize;
  dict[kNoiseSize] = noiseSize_;
  dict[kRadius] = ssaoData_.radius;
  dict[kDistribution] = hemisphereDistribution_;

  auto blurDict = blurGraph_->getPass("GaussianBlur")->getScriptingDictionary();
  dict[kBlurKernelWidth] = (uint32_t)blurDict["kernelWidth"];
  dict[kBlurSigma] = (float)blurDict["sigma"];
  return dict;
}

RenderPassReflection SSAO::reflect(const CompileData& compileData) {
  RenderPassReflection reflector;
  reflector.addInput(kColorIn, "Color buffer");
  reflector.addOutput(kColorOut, "Color-buffer with AO applied to it");
  reflector.addInput(kDepth, "Depth-buffer");
  reflector.addInput(kNormals, "World space normals, [0, 1] range")
      .flags(RenderPassReflection::Field::Flags::Optional);
  reflector.addInternal(kAoMap, "AO Map");
  return reflector;
}

void SSAO::compile(RenderContext* renderContext, const CompileData& compileData) {
  Fbo::Desc fboDesc;
  fboDesc.setColorTarget(0, Falcor::ResourceFormat::R8Unorm);
  aoFBO_ = Fbo::create2D(aoMapSize_.x, aoMapSize_.y, fboDesc);

  setKernel();
  setNoiseTexture(noiseSize_.x, noiseSize_.y);
}

void SSAO::renderDepthNormalsPrepass(
    RenderContext* renderContext,
    const Fbo::SharedPtr& targetFbo) {
  static constexpr const float4 kClearColor(0.38f, 0.52f, 0.10f, 1);
  renderContext->clearFbo(targetFbo.get(), kClearColor, 1.0f, 0, FboAttachmentType::All);
  // Per-eye should already work here if the FBO has an attachment per eye.
  //depthNormalsPrepass_->renderScene(renderContext, targetFbo, [&](EyeType eye) {});
  depthNormalsPrepass_->renderScene(renderContext, targetFbo);
}

void SSAO::renderAOBlur(
    RenderContext* renderContext,
    Texture::SharedPtr& inputTex,
    Texture::SharedPtr& outputTex) {
  auto vars = computePass_->getVars();
  vars["inTex"] = inputTex;
  vars["outTex"] = outputTex;

  auto cb = vars["perFrameConstantBuffer"];

  // TODO
  cb["kernelSize"] = 5;

  constexpr float4 kClearVal = {0.0f, 0.0f, 0.0f, 0.0f};

  renderContext->clearTexture(outputTex.get(), kClearVal);
  computePass_->execute(renderContext, {inputTex->getWidth(), inputTex->getHeight(), 1});
}

void SSAO::init() {
  Fbo::Desc fboDesc;
  fboDesc.setColorTarget(0, Falcor::ResourceFormat::R8Unorm, true);
  aoFBO_ = Fbo::create2D(aoMapSize_.x, aoMapSize_.y, fboDesc);

  uint32_t width = aoMapSize_.x;
  uint32_t height = aoMapSize_.y;

  depthNormalsFbo_ = Fbo::create();
  blurredSSAOFbo_ = Fbo::create();

  //for (EyeType eye : kAllEyes) {
    texNormals_[0] = Texture::create2D(
        width,
        height,
        ResourceFormat::RGBA8UnormSrgb,
        1,
        1,
        nullptr,
        Resource::BindFlags::ShaderResource | Resource::BindFlags::RenderTarget);

    texDepth_[0] = Texture::create2D(
        width,
        height,
        ResourceFormat::D32Float,
        1,
        1,
        nullptr,
        Resource::BindFlags::ShaderResource | Resource::BindFlags::DepthStencil);

    texBlurredAO_[0] = Texture::create2D(
        width,
        height,
        ResourceFormat::R32Float,
        1,
        1,
        nullptr,
        Resource::BindFlags::UnorderedAccess | Resource::BindFlags::ShaderResource |
            Resource::BindFlags::RenderTarget);

    // Attach textures for each eye to FBO
    depthNormalsFbo_->attachColorTarget(texNormals_[0], 0, 0, 0, Fbo::kAttachEntireMipLevel);
    depthNormalsFbo_->attachDepthStencilTarget(
        texDepth_[0], 0, 0, Fbo::kAttachEntireMipLevel);
    blurredSSAOFbo_->attachColorTarget(
        texBlurredAO_[0], 0, 0, 0, Fbo::kAttachEntireMipLevel);
  //}

  setKernel();
  setNoiseTexture(noiseSize_.x, noiseSize_.y);
}

void SSAO::generateAOMap(
    RenderContext* renderContext,
    const Camera* camera,
    uint32_t eye) {
  if (isDirty_) {
    ShaderVar var = ssaoPass_["StaticCB"];
    if (var.isValid())
      var.setBlob(ssaoData_);
    isDirty_ = false;
  }

  {
    ShaderVar var = ssaoPass_["PerFrameCB"];
    camera->setShaderData(var["gCamera"]);
  }

  renderDepthNormalsPrepass(renderContext, depthNormalsFbo_);

  // Update state/vars
  ssaoPass_["gNoiseSampler"] = noiseSampler_;
  ssaoPass_["gTextureSampler"] = textureSampler_;
  ssaoPass_["gDepthTex"] = depthNormalsFbo_->getDepthStencilTexture();
  ssaoPass_["gNoiseTex"] = noiseTexture_;
  ssaoPass_["gNormalTex"] = depthNormalsFbo_->getColorTexture(eye);

  // Generate AO
  ssaoPass_->execute(renderContext, aoFBO_);

  // NOTE/TODO: this currently breaks for VR, but not that important right now.
  renderAOBlur(renderContext, aoFBO_->getColorTexture(0), texBlurredAO_[eye]);
}

void SSAO::renderUI(Gui::Widgets& widget) {
  uint32_t distribution = (uint32_t)hemisphereDistribution_;
  if (widget.dropdown("Kernel Distribution", kDistributionDropdown, distribution))
    setDistribution(distribution);

  uint32_t size = ssaoData_.kernelSize;
  if (widget.var("Kernel Size", size, 1u, SSAOData::kMaxSamples))
    setKernelSize(size);

  float radius = ssaoData_.radius;
  if (widget.var("Sample Radius", radius, 0.001f, FLT_MAX, 0.001f))
    setSampleRadius(radius);

  widget.checkbox("Apply Blur", applyBlur_);
  if (applyBlur_) {
    auto blurGroup = Gui::Group(widget, "Blur Settings");
    if (blurGroup.open()) {
      blurGraph_->getPass("GaussianBlur")->renderUI(blurGroup);
      blurGroup.release();
    }
  }
}

void SSAO::setSampleRadius(float radius) {
  ssaoData_.radius = radius;
  isDirty_ = true;
}

void SSAO::setKernelSize(uint32_t kernelSize) {
  kernelSize = glm::clamp(kernelSize, 1u, SSAOData::kMaxSamples);
  ssaoData_.kernelSize = kernelSize;
  setKernel();
}

void SSAO::setDistribution(uint32_t distribution) {
  hemisphereDistribution_ = (SampleDistribution)distribution;
  setKernel();
}

void SSAO::setKernel() {
  for (uint32_t i = 0; i < ssaoData_.kernelSize; i++) {
    // Hemisphere in the Z+ direction
    float3 p;
    switch (hemisphereDistribution_) {
      case SampleDistribution::Random:
        p = glm::normalize(glm::linearRand(float3(-1.0f, -1.0f, 0.0f), float3(1.0f, 1.0f, 1.0f)));
        break;

      case SampleDistribution::UniformHammersley:
        p = hammersleyUniform(i, ssaoData_.kernelSize);
        break;

      case SampleDistribution::CosineHammersley:
        p = hammersleyCosine(i, ssaoData_.kernelSize);
        break;
    }

    ssaoData_.sampleKernel[i] = float4(p, 0.0f);

    // Skew sample point distance on a curve so more cluster around the origin
    float dist = (float)i / (float)ssaoData_.kernelSize;
    dist = glm::mix(0.1f, 1.0f, dist * dist);
    ssaoData_.sampleKernel[i] *= dist;
  }

  isDirty_ = true;
}

void SSAO::setNoiseTexture(uint32_t width, uint32_t height) {
  std::vector<uint32_t> data;
  data.resize(width * height);

  for (uint32_t i = 0; i < width * height; i++) {
    // Random directions on the XY plane
    float2 dir = glm::normalize(glm::linearRand(float2(-1), float2(1))) * 0.5f + 0.5f;
    data[i] = glm::packUnorm4x8(float4(dir, 0.0f, 1.0f));
  }

  noiseTexture_ = Texture::create2D(
      width, height, ResourceFormat::RGBA8Unorm, 1, Texture::kMaxPossible, data.data());

  ssaoData_.noiseScale = float2(aoFBO_->getWidth(), aoFBO_->getHeight()) / float2(width, height);

  isDirty_ = true;
}
