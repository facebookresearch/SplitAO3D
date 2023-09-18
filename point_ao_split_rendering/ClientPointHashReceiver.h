/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#pragma once

#include <Falcor.h>
#include "NetworkClient.h"
#include "HashFunctionShared.slang"
#include "PointData.slang"
#include "NetworkCompressionBase.h"

namespace split_rendering {

class ClientPointHashReceiver {
 public:
  void init();

  void receive(TCPMessage& message, Falcor::RenderContext* renderContext);

  bool isInitialized() {
    return initDone_;
  }

  Falcor::Buffer::SharedPtr& getGPUHashToPointCell() {
    return gpuHashToPointCell_;
  }

  Falcor::Buffer::SharedPtr& getGPUInstanceHashInfo() {
    return gpuInstanceHashInfo_;
  }

  Falcor::Buffer::SharedPtr& getGPUInstancePointInfo() {
    return gpuInstancePointInfo_;
  }

  Falcor::Buffer::SharedPtr& getGPUCompressedClientPointCells() {
    return gpuCompressedClientPointCells_;
  }

  Falcor::Buffer::SharedPtr& getGPUDiskRadiusPerInstance() {
    return gpuPoissonDiskRadius_;
  }

 private:
  void updateCells(TCPMessage& message, Falcor::RenderContext* renderContext);

  void updateHash(TCPMessage& message, Falcor::RenderContext* renderContext);

  Falcor::Buffer::SharedPtr gpuHashToPointCell_;
  Falcor::Buffer::SharedPtr gpuInstanceHashInfo_;
  Falcor::Buffer::SharedPtr gpuInstancePointInfo_;
  Falcor::Buffer::SharedPtr gpuCompressedClientPointCells_;
  Falcor::Buffer::SharedPtr gpuPoissonDiskRadius_;

  bool initDone_ = false;

  Falcor::ComputePass::SharedPtr cellComputePass_;
  Falcor::ComputePass::SharedPtr hashComputePass_;
  Falcor::Buffer::SharedPtr cellUpdateBuffer_;
  Falcor::Buffer::SharedPtr hashUpdateBuffer_;

  std::vector<uint8_t> decompressedData_;
  std::unique_ptr<NetworkCompressionBase> networkCompression_;
};

} // namespace split_rendering
