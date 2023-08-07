// (c) Meta Platforms, Inc. and its affiliates
#pragma once
#include "Falcor.h"
namespace split_rendering {
enum class TCPMessageType : uint8_t {
  TextMessage = 0,
  FullScreenAmbientOcclusionTextureLeft,
  FullScreenAmbientOcclusionTextureRight,
  CameraPoseMessage,
  HmdStateMessage,
  ClientResolutionChangeMessage,
  LatencyMeasureMessage,
  PAOServerInstanceHashInfo,
  PAOInstancePointInfo,
  PAOInstanceToPoissonRadius,
  PAOCompressedClientAOPoints,
  PAOServerHashToPointCell,
  PAOEndOfInit,
  PAOPointCellUpdate,
  PAOHashUpdate,
  NumberOfMessageTypes // Keep this last.
};

struct TCPMessageHeader {
  uint32_t id; // id for referencing speficic packets and/or debugging
  uint32_t size; // size of the payload
  uint32_t decompressedSize; // size of the uncompressed payload
  uint32_t width;
  uint32_t height;
  float timestamp;
  TCPMessageType type; // type of the payload
  // more stuff here if we need it
};

struct TCPMessage {
  TCPMessageHeader header;
  std::vector<uint8_t> data;

  TCPMessage::TCPMessage()
      : header{0, 0, 0, 0, 0, 0.0f, TCPMessageType::NumberOfMessageTypes}, data(0){};
};

struct CameraPoseData {
  Falcor::float3 headPos;
  Falcor::float3 upVec;
  Falcor::float3 headTarget;
};
} // namespace split_rendering
