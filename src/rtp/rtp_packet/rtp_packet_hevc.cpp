#include "rtp_packet_hevc.h"

namespace minirtc {

RtpPacketHevc::RtpPacketHevc() = default;

RtpPacketHevc::~RtpPacketHevc() = default;

bool RtpPacketHevc::GetFrameHeaderInfo() {
  if (header_parsed_) {
    return true;
  }

  const uint8_t* frame_buffer = Payload();
  if (!frame_buffer) {
    return false;
  }

  size_t offset = 0;
  if (rtp::PAYLOAD_TYPE::RTX == PayloadType()) {
    if (PayloadSize() < 2) {
      return false;
    }
    osn_ = static_cast<uint16_t>((frame_buffer[0] << 8) | frame_buffer[1]);
    offset = 2;
  }

  if (PayloadSize() < offset + 2) {
    return false;
  }

  const uint8_t flags = frame_buffer[offset];
  start_ = (flags & 0x80u) != 0;
  end_ = (flags & 0x40u) != 0;
  keyframe_ = (flags & 0x20u) != 0;

  add_offset_to_payload(offset + 2);
  header_parsed_ = true;
  return true;
}

}  // namespace minirtc
