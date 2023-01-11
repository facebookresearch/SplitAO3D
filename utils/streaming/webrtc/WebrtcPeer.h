// (c) Meta Platforms, Inc. and its affiliates. Confidential and proprietary.

#pragma once

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

// This is file contians defintion of the WebRTC-based Streaming API developed
// by FRL Graphics team. It is intended to be used by Cloud Rendering System
// (potentially other FRL projects in the future) to achieve secure,
// low-latency, real-time streaming of video, audio and data. It might get
// extended in the future to support streaming of other graphic assets, such as
// used in split rendering. See API design doc here for more info:
// https://docs.google.com/document/d/1liLYjO6qui9IYy9sn4APtQogE1SHCsrYZ-Y5uj02uSs/edit#heading=h.yxo4a878135

// rlr_streaming: Reality Lab Research Streaming
namespace rlr_streaming {

enum struct WebrtcEvent {
  // For notifying app about the state w.r.t. connection with singaling server.
  SignalingServerSignedIn,
  // For notifying app about the new state change of peer connection.
  PeerConnectionNew,
  PeerConnectionConnecting,
  PeerConnectionConnected, // peer connection ready to send media
  PeerConnectionDisconnected,
  PeerConnectionFailed,
  PeerConnectionClosed,
  // For notifying app about the new state change of data channel.
  DataChannelConnecting,
  DataChannelOpen, // data channel is ready to send data
  DataChannelClosing,
  DataChannelClosed,
};

// Data struct for |SignalingServerSignedIn| event.
struct SignalingServerSignedInData {
  // ID assigned by signaling server to the app/peer.
  int64_t id;
};

// Apps that intends to uses the Streaming library needs to implement this
// interface and pass the object to |WebrtcPeer| during construction. The
// implementation is used to handle event notifications from the Streaming
// library. For instance, app should not try to send media data until being
// notified that peer connection state has transitioned to |kConnected|, and app
// should not  try to send data using the data channel until being notified that
// data channel has been established.
class WebrtcEventHandler {
 public:
  virtual ~WebrtcEventHandler() = default;
  // When |data| is not nullptr, it points to a struct defined per each type of event.
  // E.g., |SignalingServerSignedInData| for |SignalingServerSignedIn| event, where application can
  // cast |data| to a |SignalingServerSignedInData| to access the additional data returned for that
  // type of event. If there is no struct defined for a event type, that means that event is not
  // returned with any additional data.
  virtual void onEvent(WebrtcEvent event, void* data) = 0;
};

// App can set |iceProtocol| in the |WebrtcPeerInit| to configure WebRTC to use
// a specific transport layer protocol (UDP/TCP), or asks WebRTC to not
// providing a ICE candidate (NONE) in which case only when ICE candidate from
// the remote peer will be used for peer connection.
enum struct IceProtocol {
  // Any protocol (UDP or TCP) is fine. UDP is the default.
  Any,
  // Only use UDP ICE candidate.
  Udp,
  // Only use TCP ICE candidate.
  Tcp,
  // Do not share any ICE candidate to reomte peer,
  // so no connection from remote peer
  // only connection to remote peer
  None,
};

// App can set |videoCodec| in the |WebrtcPeerInit| to configure WebRTC to use
// a specific video codec for video encoding.
enum struct VideoCodec {
  VideoCodecUnspecified,
  Vp8,
  Vp9,
  Av1,
  H264,
};

enum struct ColorFormat {
  ColorFormatUnspecified,
  Rgb,
  Rgba,
};

static const int kAddrLen = 100;
static const int kFolderLen = 500;

// This struct carries various configuration parameters supported by the
// Streaming API. App should pass in an instance of this struct to WebRTC init
// call to configure the behavior of WebRTC prior to using the API to exchange
// media/data.
struct WebrtcPeerInit {
  // NOTE: Using char array instead of string type because when
  // passing in WebrtcPeerInit from Blender string is causing crash.
  // NOTE: Be careful when creating new char array field. It has caused expected crash in
  // PhysicalSocketServer::PhysicalSocketServer() before. We support two methods of exchanging
  // signaling messages (SDP, ICE) to bootstrap the peer connection.
  // (1) Via a signaling server. In this case a signaling server should be launched first. Then
  // |signalingServerAddress| and |signalingServerPort| needs to be provided to both peers.
  // Furthermore, the peer initiating the connection needs to know the ID of the remote peer, i.e.,
  // |peerIdToConnect|.
  // (2) Via direct connection from one peer to another remote peer. In this case
  // one peer should be listening on a TCP |signalingPort|, and the other peer needs to have
  // |remotePeerAddress| and |remotePeerPort|. Only one of the two methods should be used. To
  // support signaling method (1)
  char signalingServerAddress[kAddrLen] = "";
  int64_t signalingServerPort = 0;
  int64_t peerIdToConnect = -1;
  // To support signaling method (2)
  bool signalingIpV6 = true; // server peer only
  int64_t signalingPort = 0; // server peer only
  char remotePeerAddress[kAddrLen] = ""; // client peer only
  int64_t remotePeerPort = 0; // client peer only

  // The protocol used for P2P connection.
  IceProtocol iceProtocol = IceProtocol::Any;
  // The port used for P2P connection.
  // Default is 0, meaning random port will be selected.
  int64_t icePort = 0;

  // Whether to create a video track attached to the peer connection.
  bool createVideoTrack = false;
  // Whether to create a data channel attached to the peer connection.
  bool createDataChannel = false;
  // Per api/data_channel_interface.h
  // Data channel reliability is assumed, and channel will be unreliable if
  // DataChannelInit.maxRetransmitTime or DataChannelInit.maxRetransmits is set.
  // DEPRECATED use individual field for each fieldTrial
  // char fieldTrials[100] = "";
  // Whether the video is stereo.
  // Setting this to true only has an impact on the receiver end.
  // Received video frame will be splitted into two frames (left and right eye).
  bool stereo = false;

  // When set to bigger than 0, "offer" session description sent by client will
  // be updated to assign min bitrate to this bitrate kbps
  int64_t minBitrateKbps = -1;
  // When set to bigger than 0, "offer" session description sent by client will
  // be updated to assign max bitrate to this bitrate kbps
  int64_t maxBitrateKbps = -1;

  // "offer" session description sent by client will be updated with this
  // |videoCodec|, in order to ask master to use this codec when streaming
  VideoCodec videoCodec = VideoCodec::Vp8;
  // We only support NvEncode right now, which will be used when this is true.
  bool hardwareAccelerated = false;

  // TODO(fangy): Consider support different sendFrameFormat as well.
  ColorFormat receiveFrameFormat = ColorFormat::Rgb;

  // When set to bigger than 0, peer connection stats will be pulled ever
  // |statsCollectIntervalMs| milliseconds.
  // Set it to be > 1000 (1 sec).
  int64_t statsCollectIntervalMs = 0;
  // Write log lines for latency analysis
  bool measureLatency = false;

  // Save send / received video frames in ivf files for debugging
  bool writeIvfFile = false;
  // Parameters that will trigger frame output to a folder
  // to support streaming quality evaluation
  char sendFrameOutputFolder[kFolderLen] = "";
  float sendFrameOutputSampleRate = 0.0;
  char receiveFrameOutputFolder[kFolderLen] = "";
  float receiveFrameOutputSampleRate = 0.0;
};

// Class holding info about a frame sent/received via WebRTC.
class WebrtcFrame {
 public:
  WebrtcFrame(uint16_t id, const uint8_t* frameBuffer, size_t width, size_t height)
      : id_(id), frameBuffer_(frameBuffer), width_(width), height_(height) {}
  ~WebrtcFrame() = default;
  uint16_t id() const {
    return id_;
  }
  const uint8_t* frameBuffer() const {
    return frameBuffer_;
  }
  size_t width() const {
    return width_;
  }
  size_t height() const {
    return height_;
  }
  size_t bitDepth() const {
    return bitDepth_;
  }
  size_t channelCount() const {
    return channelCount_;
  }

 private:
  uint16_t id_ = 0;
  const uint8_t* frameBuffer_;
  size_t width_;
  size_t height_;
  // Currently we only support 8 bits RGB image. To be extended later.
  size_t bitDepth_ = 8;
  size_t channelCount_ = 3;
};

// App that usese the Streaming API to receive video needs to implement this
// interface. The implementation will be notified when a new frame becomes
// avaialbe.
class WebrtcFrameSink {
 public:
  virtual ~WebrtcFrameSink() = default;
  // |frame.frameBuffer| is owned by the WebrtcPeer implementation.
  // Caller should use / copy |frame.frameBuffer| during onFrame() call.
  // Caller should not access |frame.frameBuffer| after onFrame() has returned.
  virtual void onFrame(const WebrtcFrame& frame) = 0;
  // When WebrtcPeerInit.stereo is set to true,
  // onFrame() will not be called, instead onStereoFrames() will be called
  // and the received frame (stacked of left eye and right eye frames)
  // will be spitted back to two frames and returned.
  virtual void onStereoFrames(const WebrtcFrame& frameLeft, const WebrtcFrame& frameRight) {
    std::cout
        << "Error: onStereoFrames() default implementation is invoked, you need to override this method to receive stereo frames"
        << std::endl;
  }
};

// App that usese the Streaming API to receive data via the data channel needs
// to implement this interface. The implementation will be notified when a new
// data message becomes avaialbe.
class WebrtcDataSink {
 public:
  virtual ~WebrtcDataSink() = default;
  virtual void onData(const void* dataBuffer, size_t bufferSize) = 0;
};

// When |statsCollectIntervalMs| in |WebrtcPeerInit| is set to bigger than 0,
// Streaming library will collect internal WebRTC stats on the peer connection
// in the frequency defined by |statsCollectIntervalMs|. App can receive such
// stats whenever it collected via implmenting this |WebrtcStatsSink| and pass
// the object to |WebrtcPeer| by calling addDataSink().
class WebrtcStatsSink {
 public:
  virtual ~WebrtcStatsSink() = default;
  // Return the stats json string as char array.
  // Do not use std::string as string does not work across .so boundry.
  virtual void onStats(const char* statsBuffer, size_t statsSize) const = 0;
};

// Forward declaration to hide the declaration from
// WebrtcPeer.h users.
class WebrtcPeerImpl;

// This is the main interface of Streaming API.
// App will create and initialize an instance of this class and use to
// send/receive video and send/receive data. This class not thread-safe.
class WebrtcPeer {
 public:
  WebrtcPeer() = delete;
  // |handler| should have a lifetime that exceeds that of WebrtcPeer
  WebrtcPeer(WebrtcEventHandler* handler);
  ~WebrtcPeer();

  // init() will make WebrtcPeer to connect to the signaling server.
  // This is non-trivial work, hence it is done in init(), not in the ctor.
  // Also it supports such init work to be performed in later time
  // than when WebrtcPeer object is created, say certain condition is met.
  // TODO(fangy): Consider renaming to connectToSignalingServer();
  void init(const WebrtcPeerInit& init);
  // Intended to be called by cloud rendering master, to allow new peer
  // connection to be created with a new client.
  void deletePeerConnection();

  // Send video frame.
  // Return value indicates whether the conversion from RGB to YUV failed.
  // Caller can continue to use sendFrame() to send other frames.
  bool sendFrame(const WebrtcFrame& frame);

  // Stereo streaming.
  bool sendStereoFrames(const WebrtcFrame& frameLeft, const WebrtcFrame& frameRight);

  // Receive video frame.
  void addFrameSink(WebrtcFrameSink* sink);
  void removeFrameSink(WebrtcFrameSink* sink);

  // Send arbitrary data.
  // Return false if data channel has not been created.
  // Until data channel is created, subsequent calls to
  // sendData() will continue to fail.
  // In our experiment, max bufferSize we can send is 256 KB
  // https://docs.google.com/document/d/1ISj6yws0ePWmkYV6mwL1e_v_SYu6heuIltrbHf4x3o4/edit
  // even though some online reference says such limit is 64 KB
  // https://stackoverflow.com/questions/15435121/what-is-the-maximum-size-of-webrtc-data-channel-messages
  // When you send data exceeding 256 KB, data channel will close.
  bool sendData(const void* dataBuffer, size_t bufferSize);

  // Receive arbitrary data.
  void addDataSink(WebrtcDataSink* sink);
  void removeDataSink(WebrtcDataSink* sink);

  // Receive stats.
  // WebrtcPeerInit.statsCollectIntervalMs needs to > 0
  // for stats collect tasks to run.
  void addStatsSink(WebrtcStatsSink* sink);
  void removeStatsSink(WebrtcStatsSink* sink);

  // Utilities.

  // Do not return std::string as this causes linking issue with Blender.
  static void eventToString(const WebrtcEvent& event, char* str, size_t strSize);

 protected:
  std::unique_ptr<WebrtcPeerImpl> impl_;
};

} // namespace rlr_streaming
