// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#pragma once

#include <gflags/gflags.h>

// Signaling via a signaling server.
DEFINE_string(signaling_server_address, "", "");
DEFINE_int64(signaling_server_port, 0, "");
DEFINE_int64(peer_id_to_connect, 0, ""); // client only

// Server only.
DEFINE_string(rgb_file, "", "");
DEFINE_int64(width, 0, "");
DEFINE_int64(height, 0, "");

// Client only.
DEFINE_string(video_codec, "", "");
DEFINE_int64(min_bitrate_kbps, 0, "");
DEFINE_int64(max_bitrate_kbps, 0, "");

// Both server and client.
DEFINE_string(ice_protocol, "", "");
DEFINE_int64(ice_port, 0, "");
DEFINE_int64(send_interval_ms, 1000, "");