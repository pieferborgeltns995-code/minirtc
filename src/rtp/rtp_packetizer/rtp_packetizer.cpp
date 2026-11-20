#include "rtp_packetizer.h"

#include <algorithm>

#include "rtp_packetizer_av1.h"
#include "rtp_packetizer_generic.h"
#include "rtp_packetizer_h264.h"
#include "rtp_header_extension.h"

namespace minirtc {

std::unique_ptr<RtpPacketizer> RtpPacketizer::Create(uint32_t payload_type,
                                                     uint32_t ssrc) {
  switch (payload_type) {
    case rtp::PAYLOAD_TYPE::H264:
      return std::make_unique<RtpPacketizerH264>(ssrc);
    case rtp::PAYLOAD_TYPE::AV1:
      return std::make_unique<RtpPacketizerAv1>(ssrc);
    default:
      return std::make_unique<RtpPacketizerGeneric>(ssrc, payload_type);
  }
}

void RtpPacketizer::SetAbsoluteSendTimeExtensionId(
    std::optional<uint8_t> extension_id) {
  if (extension_id.has_value() &&
      !rtp::IsValidOneByteExtensionId(*extension_id)) {
    abs_send_time_ext_id_.reset();
    return;
  }
  abs_send_time_ext_id_ = extension_id;
}

void RtpPacketizer::AppendAbsoluteSendTimeExtension(
    std::vector<uint8_t>& rtp_packet_frame) const {
  if (!abs_send_time_ext_id_.has_value()) {
    return;
  }

  rtp_packet_frame.push_back(
      static_cast<uint8_t>(kOneByteExtensionProfileId >> 8));
  rtp_packet_frame.push_back(
      static_cast<uint8_t>(kOneByteExtensionProfileId));
  rtp_packet_frame.push_back(0);
  rtp_packet_frame.push_back(1);

  constexpr uint8_t kAbsoluteSendTimeLengthMinusOne = 2;
  rtp_packet_frame.push_back(
      static_cast<uint8_t>((*abs_send_time_ext_id_ << 4) |
                           kAbsoluteSendTimeLengthMinusOne));
  // Reserve the wire bytes now so pacing sees the final packet size. The
  // actual Q6.18 value is written immediately before transport send.
  rtp_packet_frame.insert(rtp_packet_frame.end(), 3, 0);
}

std::vector<std::unique_ptr<RtpPacket>> RtpPacketizer::BuildPaddingPackets(
    uint32_t payload_size, uint32_t rtp_timestamp, uint8_t padding_payload_type,
    uint32_t ssrc, uint16_t& sequence_number,
    const std::vector<uint32_t>& csrcs, uint32_t csrc_count,
    bool use_rtp_packet_to_send, std::vector<uint8_t>& rtp_packet_frame) const {
  std::vector<std::unique_ptr<RtpPacket>> rtp_packets;

  // The RTP padding count is an 8-bit field. Keep one standards-compliant
  // padding byte at the end and carry the rest as probe payload under the
  // dedicated padding payload type, so large probes do not explode into
  // hundreds of tiny packets.
  constexpr uint8_t kRtpPaddingSize = 1;
  const bool has_extension = HasAbsoluteSendTimeExtension();
  uint32_t remaining_size = payload_size;
  while (remaining_size > 0) {
    const uint32_t packet_data_size =
        std::min<uint32_t>(remaining_size, max_payload_size_);
    ++sequence_number;

    rtp_packet_frame.clear();
    rtp_packet_frame.push_back((kRtpVersion << 6) | (1 << 5) |
                               (has_extension << 4) | 0);
    rtp_packet_frame.push_back(padding_payload_type & 0x7F);
    rtp_packet_frame.push_back((sequence_number >> 8) & 0xFF);
    rtp_packet_frame.push_back(sequence_number & 0xFF);
    rtp_packet_frame.push_back((rtp_timestamp >> 24) & 0xFF);
    rtp_packet_frame.push_back((rtp_timestamp >> 16) & 0xFF);
    rtp_packet_frame.push_back((rtp_timestamp >> 8) & 0xFF);
    rtp_packet_frame.push_back(rtp_timestamp & 0xFF);
    rtp_packet_frame.push_back((ssrc >> 24) & 0xFF);
    rtp_packet_frame.push_back((ssrc >> 16) & 0xFF);
    rtp_packet_frame.push_back((ssrc >> 8) & 0xFF);
    rtp_packet_frame.push_back(ssrc & 0xFF);

    for (uint32_t index = 0; index < csrc_count && !csrcs.empty(); ++index) {
      rtp_packet_frame.push_back((csrcs[index] >> 24) & 0xFF);
      rtp_packet_frame.push_back((csrcs[index] >> 16) & 0xFF);
      rtp_packet_frame.push_back((csrcs[index] >> 8) & 0xFF);
      rtp_packet_frame.push_back(csrcs[index] & 0xFF);
    }

    if (has_extension) {
      AppendAbsoluteSendTimeExtension(rtp_packet_frame);
    }

    rtp_packet_frame.insert(rtp_packet_frame.end(), packet_data_size, 0);
    rtp_packet_frame.back() = kRtpPaddingSize;

    if (use_rtp_packet_to_send) {
      auto rtp_packet = std::make_unique<webrtc::RtpPacketToSend>();
      if (rtp_packet->Build(rtp_packet_frame.data(), rtp_packet_frame.size())) {
        rtp_packets.emplace_back(std::move(rtp_packet));
      }
    } else {
      auto rtp_packet = std::make_unique<RtpPacket>();
      if (rtp_packet->Build(rtp_packet_frame.data(), rtp_packet_frame.size())) {
        rtp_packets.emplace_back(std::move(rtp_packet));
      }
    }

    remaining_size -= packet_data_size;
  }

  return rtp_packets;
}
}  // namespace minirtc
