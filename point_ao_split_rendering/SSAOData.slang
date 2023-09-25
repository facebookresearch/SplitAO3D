/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#pragma once
#include "Utils/HostDeviceShared.slangh"

BEGIN_NAMESPACE_FALCOR

struct SSAOData
{
    static const uint32_t kMaxSamples = 32;

    float4 sampleKernel[kMaxSamples];
    float2 noiseScale = float2(1, 1);
    uint32_t kernelSize = 16;
    float radius = 0.1f;
};

END_NAMESPACE_FALCOR
