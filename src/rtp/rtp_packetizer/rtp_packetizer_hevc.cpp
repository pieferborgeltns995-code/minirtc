#include "rtp_packetizer_hevc.h"

#include <algorithm>

namespace minirtc {

RtpPacketizerHevc::RtpPacketizerHevc(uint32_t ssrc)
    : payload_type_(rtp::PAYLOAD_TYPE::HEVC), ssrc_(ssrc) {}

RtpPacketizerHevc::~RtpPacketizerHevc() = default;

std::vector<std::unique_ptr<RtpPacket>> RtpPacketizerHevc::Build(
    uint8_t* payload, uint32_t payload_size, uint32_t rtp_timestamp,
    bool use_rtp_packet_to_send) {
  std::lock_guard<std::mutex> lock(packetizer_mutex_);
  std::vector<std::unique_ptr<RtpPacket>> rtp_packets;
  if (!payload || payload_size == 0 || max_payload_size_ == 0) {
    return rtp_packets;
  }

  const bool has_extension = HasAbsoluteSendTimeExtension();
  const uint32_t packet_num =
      (payload_size + max_payload_size_ - 1u) / max_payload_size_;
  rtp_packets.reserve(packet_num);

  for (uint32_t index = 0; index < packet_num; ++index) {
    const bool is_first = (index == 0);
    const bool is_last = (index == packet_num - 1u);
    const uint32_t offset = index * max_payload_size_;
    const uint32_t chunk_size =
        std::min<uint32_t>(max_payload_size_, payload_size - offset);

    ++sequence_number_;
    const uint8_t marker = is_last ? 1u : 0u;
    const uint8_t flags = static_cast<uint8_t>(
        (is_first ? 0x80u : 0x00u) | (is_last ? 0x40u : 0x00u) |
        ((is_first && current_frame_is_key_frame_) ? 0x20u : 0x00u));

    rtp_packet_frame_.clear();
    rtp_packet_frame_.reserve(12u + (has_extension ? 8u : 0u) + 2u +
                              chunk_size);
    rtp_packet_frame_.push_back(static_cast<uint8_t>(
        (kRtpVersion << 6) | (has_extension ? (1u << 4) : 0u)));
    rtp_packet_frame_.push_back(
        static_cast<uint8_t>((marker << 7) | (payload_type_ & 0x7Fu)));
    rtp_packet_frame_.push_back(
        static_cast<uint8_t>((sequence_number_ >> 8) & 0xFFu));
    rtp_packet_frame_.push_back(static_cast<uint8_t>(sequence_number_ & 0xFFu));
    rtp_packet_frame_.push_back(
        static_cast<uint8_t>((rtp_timestamp >> 24) & 0xFFu));
    rtp_packet_frame_.push_back(
        static_cast<uint8_t>((rtp_timestamp >> 16) & 0xFFu));
    rtp_packet_frame_.push_back(
        static_cast<uint8_t>((rtp_timestamp >> 8) & 0xFFu));
    rtp_packet_frame_.push_back(static_cast<uint8_t>(rtp_timestamp & 0xFFu));
    rtp_packet_frame_.push_back(static_cast<uint8_t>((ssrc_ >> 24) & 0xFFu));
    rtp_packet_frame_.push_back(static_cast<uint8_t>((ssrc_ >> 16) & 0xFFu));
    rtp_packet_frame_.push_back(static_cast<uint8_t>((ssrc_ >> 8) & 0xFFu));
    rtp_packet_frame_.push_back(static_cast<uint8_t>(ssrc_ & 0xFFu));

    if (has_extension) {
      AppendAbsoluteSendTimeExtension(rtp_packet_frame_);
    }

    rtp_packet_frame_.push_back(flags);
    rtp_packet_frame_.push_back(0x00u);
    rtp_packet_frame_.insert(rtp_packet_frame_.end(), payload + offset,
                             payload + offset + chunk_size);

    if (use_rtp_packet_to_send) {
      auto rtp_packet = std::make_unique<webrtc::RtpPacketToSend>();
      if (rtp_packet->Build(rtp_packet_frame_.data(),
                            static_cast<uint32_t>(rtp_packet_frame_.size()))) {
        rtp_packets.emplace_back(std::move(rtp_packet));
      }
    } else {
      auto rtp_packet = std::make_unique<RtpPacket>();
      if (rtp_packet->Build(rtp_packet_frame_.data(),
                            static_cast<uint32_t>(rtp_packet_frame_.size()))) {
        rtp_packets.emplace_back(std::move(rtp_packet));
      }
    }
  }

  return rtp_packets;
}

std::vector<std::unique_ptr<RtpPacket>> RtpPacketizerHevc::BuildPadding(
    uint32_t payload_size, uint32_t rtp_timestamp,
    bool use_rtp_packet_to_send) {
  std::lock_guard<std::mutex> lock(packetizer_mutex_);
  std::vector<std::unique_ptr<RtpPacket>> rtp_packets;
  if (payload_size == 0 || max_payload_size_ == 0) {
    return rtp_packets;
  }

  constexpr uint8_t kRtpPaddingSize = 1;
  const bool has_extension = HasAbsoluteSendTimeExtension();
  const uint8_t padding_pt = static_cast<uint8_t>((payload_type_ - 1u) & 0x7Fu);

  uint32_t remaining_size = payload_size;
  while (remaining_size > 0) {
    const uint32_t packet_data_size =
        std::min<uint32_t>(remaining_size, max_payload_size_);
    ++sequence_number_;

    rtp_packet_frame_.clear();
    rtp_packet_frame_.reserve(12u + (has_extension ? 8u : 0u) +
                              packet_data_size);
    rtp_packet_frame_.push_back(static_cast<uint8_t>(
        (kRtpVersion << 6) | (1u << 5) | (has_extension ? (1u << 4) : 0u)));
    rtp_packet_frame_.push_back(padding_pt);
    rtp_packet_frame_.push_back(
        static_cast<uint8_t>((sequence_number_ >> 8) & 0xFFu));
    rtp_packet_frame_.push_back(static_cast<uint8_t>(sequence_number_ & 0xFFu));
    rtp_packet_frame_.push_back(
        static_cast<uint8_t>((rtp_timestamp >> 24) & 0xFFu));
    rtp_packet_frame_.push_back(
        static_cast<uint8_t>((rtp_timestamp >> 16) & 0xFFu));
    rtp_packet_frame_.push_back(
        static_cast<uint8_t>((rtp_timestamp >> 8) & 0xFFu));
    rtp_packet_frame_.push_back(static_cast<uint8_t>(rtp_timestamp & 0xFFu));
    rtp_packet_frame_.push_back(static_cast<uint8_t>((ssrc_ >> 24) & 0xFFu));
    rtp_packet_frame_.push_back(static_cast<uint8_t>((ssrc_ >> 16) & 0xFFu));
    rtp_packet_frame_.push_back(static_cast<uint8_t>((ssrc_ >> 8) & 0xFFu));
    rtp_packet_frame_.push_back(static_cast<uint8_t>(ssrc_ & 0xFFu));

    if (has_extension) {
      AppendAbsoluteSendTimeExtension(rtp_packet_frame_);
    }

    rtp_packet_frame_.insert(rtp_packet_frame_.end(), packet_data_size, 0u);
    rtp_packet_frame_.back() = kRtpPaddingSize;

    if (use_rtp_packet_to_send) {
      auto rtp_packet = std::make_unique<webrtc::RtpPacketToSend>();
      if (rtp_packet->Build(rtp_packet_frame_.data(),
                            static_cast<uint32_t>(rtp_packet_frame_.size()))) {
        rtp_packets.emplace_back(std::move(rtp_packet));
      }
    } else {
      auto rtp_packet = std::make_unique<RtpPacket>();
      if (rtp_packet->Build(rtp_packet_frame_.data(),
                            static_cast<uint32_t>(rtp_packet_frame_.size()))) {
        rtp_packets.emplace_back(std::move(rtp_packet));
      }
    }

    remaining_size -= packet_data_size;
  }

  return rtp_packets;
}

}  // namespace minirtc
