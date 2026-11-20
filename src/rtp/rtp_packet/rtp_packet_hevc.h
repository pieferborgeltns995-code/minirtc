/*
 * Copyright (c) 2026 MiniRTC Contributors, All Rights Reserved.
 */

#ifndef _RTP_PACKET_HEVC_H_
#define _RTP_PACKET_HEVC_H_

#include <cstdint>

#include "rtp_packet.h"

namespace minirtc {

// Native MiniRTC HEVC frame-fragment RTP payload header (2 bytes):
//   Byte 0: [S(1) | E(1) | K(1) | Reserved(5)]
//   Byte 1: [Reserved(8)]
// Followed by raw Annex-B HEVC frame bytes.
class RtpPacketHevc : public RtpPacket {
 public:
  RtpPacketHevc();
  virtual ~RtpPacketHevc();

  bool GetFrameHeaderInfo();

  bool HevcFrameStart() const { return start_; }
  bool HevcFrameEnd() const { return end_; }
  bool IsKeyFrame() const { return keyframe_; }
  uint16_t GetOsn() const { return osn_; }

 private:
  uint16_t osn_ = 0;
  bool start_ = false;
  bool end_ = false;
  bool keyframe_ = false;
  bool header_parsed_ = false;
};

}  // namespace minirtc

#endif
