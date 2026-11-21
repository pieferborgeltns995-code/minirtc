/*
 * Copyright (c) 2026 MiniRTC Contributors, All Rights Reserved.
 */

#ifndef _RTP_PACKETIZER_HEVC_H_
#define _RTP_PACKETIZER_HEVC_H_

#include <cstdint>
#include <mutex>
#include <vector>

#include "rtp_packetizer.h"

namespace minirtc {

class RtpPacketizerHevc : public RtpPacketizer {
 public:
  explicit RtpPacketizerHevc(uint32_t ssrc);
  ~RtpPacketizerHevc() override;

  std::vector<std::unique_ptr<RtpPacket>> Build(
      uint8_t* payload, uint32_t payload_size, uint32_t rtp_timestamp,
      bool use_rtp_packet_to_send) override;

  std::vector<std::unique_ptr<RtpPacket>> BuildPadding(
      uint32_t payload_size, uint32_t rtp_timestamp,
      bool use_rtp_packet_to_send) override;

  void SetIsKeyFrame(bool is_key_frame) override {
    current_frame_is_key_frame_ = is_key_frame;
  }

 private:
  std::mutex packetizer_mutex_;
  uint32_t payload_type_ = rtp::PAYLOAD_TYPE::HEVC;
  uint16_t sequence_number_ = 0;
  uint32_t ssrc_ = 0;
  bool current_frame_is_key_frame_ = false;
  std::vector<uint8_t> rtp_packet_frame_;
};

}  // namespace minirtc

#endif
