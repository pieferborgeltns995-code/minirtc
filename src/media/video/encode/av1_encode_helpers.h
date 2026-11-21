#ifndef MINIRTC_MEDIA_VIDEO_ENCODE_AV1_ENCODE_HELPERS_H_
#define MINIRTC_MEDIA_VIDEO_ENCODE_AV1_ENCODE_HELPERS_H_

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <vector>

#include "log.h"
#include "media_codec.h"
#include "nv12_scaler.h"
#include "video/av1_crop_metadata.h"

namespace minirtc {
namespace av1_encode {

constexpr uint8_t kObuTypeSequenceHeader = 1;
constexpr uint8_t kObuTypeTemporalDelimiter = 2;
constexpr uint8_t kObuTypeFrameHeader = 3;
constexpr uint8_t kObuTypeFrame = 6;

inline bool ValidConfig(const MediaCodecConfig& config) {
  size_t size = 0;
  return config.init_width > 0 && config.init_height > 0 &&
         config.init_width <= 8192 && config.init_height <= 8192 &&
         GetNv12FrameSize(config.init_width, config.init_height, &size) &&
         config.init_bitrate > 0 && config.max_bitrate > 0 &&
         config.max_frame_rate > 0 && config.max_frame_rate <= 240 &&
         config.key_frame_interval > 0;
}

enum class EncodeMode {
  kNative = 0,
  kPad = 1,
  kScale = 2,
};

struct EncodeSize {
  int width = 0;
  int height = 0;
  EncodeMode mode = EncodeMode::kNative;
};

inline int AlignDimDown(int value, int align, int min_value) {
  if (value <= min_value) return min_value;
  return (value / align) * align;
}

inline int AlignDimUp(int value, int align) {
  if (value <= 0) return align;
  return ((value + align - 1) / align) * align;
}

inline std::vector<EncodeSize> BuildCandidateEncodeSizes(int src_w, int src_h) {
  std::vector<EncodeSize> candidates;
  auto push_unique = [&](int w, int h, EncodeMode mode) {
    if (w <= 0 || h <= 0 || w > 8192 || h > 8192 || (w & 1) != 0 ||
        (h & 1) != 0) {
      return;
    }
    for (const auto& c : candidates) {
      if (c.width == w && c.height == h) return;
    }
    candidates.push_back({w, h, mode});
  };
  push_unique(src_w, src_h, EncodeMode::kNative);
  // Prefer 1:1 edge-clamped padding to 64x16 alignment over scaling so desktop
  // text keeps native sharpness without bilinear resampling.
  push_unique(AlignDimUp(src_w, 64), AlignDimUp(src_h, 16), EncodeMode::kPad);
  // Fallback to downscaling if padded size exceeds hardware limits.
  push_unique(AlignDimDown(src_w, 64, 64), AlignDimDown(src_h, 16, 16),
              EncodeMode::kScale);
  if (src_w > 3840 || src_h > 2160) {
    const double scale = (std::min)(3840.0 / static_cast<double>(src_w),
                                    2160.0 / static_cast<double>(src_h));
    push_unique(AlignDimDown(static_cast<int>(src_w * scale), 64, 64),
                AlignDimDown(static_cast<int>(src_h * scale), 16, 16),
                EncodeMode::kScale);
  }
  return candidates;
}

inline bool PadNv12EdgeClampStrided(const uint8_t* src, int src_w, int src_h,
                                    uint8_t* dst_y, int dst_y_pitch,
                                    uint8_t* dst_uv, int dst_uv_pitch,
                                    int dst_w, int dst_h) {
  if (!src || !dst_y || !dst_uv || src_w <= 0 || src_h <= 0 ||
      dst_w < src_w || dst_h < src_h || dst_y_pitch < dst_w ||
      dst_uv_pitch < dst_w || (src_w & 1) != 0 || (src_h & 1) != 0 ||
      (dst_w & 1) != 0 || (dst_h & 1) != 0) {
    return false;
  }
  const int pad_right = dst_w - src_w;
  for (int y = 0; y < src_h; ++y) {
    const uint8_t* src_row = src + static_cast<size_t>(y) * src_w;
    uint8_t* dst_row = dst_y + static_cast<size_t>(y) * dst_y_pitch;
    std::memcpy(dst_row, src_row, static_cast<size_t>(src_w));
    if (pad_right > 0) {
      std::memset(dst_row + src_w, src_row[src_w - 1],
                  static_cast<size_t>(pad_right));
    }
  }
  const uint8_t* last_y =
      dst_y + static_cast<size_t>(src_h - 1) * dst_y_pitch;
  for (int y = src_h; y < dst_h; ++y) {
    std::memcpy(dst_y + static_cast<size_t>(y) * dst_y_pitch, last_y,
                static_cast<size_t>(dst_w));
  }

  const uint8_t* src_uv = src + static_cast<size_t>(src_w) * src_h;
  const int src_uv_h = src_h / 2;
  const int dst_uv_h = dst_h / 2;
  for (int y = 0; y < src_uv_h; ++y) {
    const uint8_t* src_row = src_uv + static_cast<size_t>(y) * src_w;
    uint8_t* dst_row = dst_uv + static_cast<size_t>(y) * dst_uv_pitch;
    std::memcpy(dst_row, src_row, static_cast<size_t>(src_w));
    const uint8_t u = src_row[src_w - 2];
    const uint8_t v = src_row[src_w - 1];
    for (int x = src_w; x < dst_w; x += 2) {
      dst_row[x] = u;
      dst_row[x + 1] = v;
    }
  }
  const uint8_t* last_uv =
      dst_uv + static_cast<size_t>(src_uv_h - 1) * dst_uv_pitch;
  for (int y = src_uv_h; y < dst_uv_h; ++y) {
    std::memcpy(dst_uv + static_cast<size_t>(y) * dst_uv_pitch, last_uv,
                static_cast<size_t>(dst_w));
  }
  return true;
}

inline bool PrepareNv12ToStridedPlanes(
    const uint8_t* src_nv12, int src_w, int src_h, uint8_t* dst_y,
    int dst_y_pitch, uint8_t* dst_uv, int dst_uv_pitch, int dst_w, int dst_h,
    EncodeMode mode, std::vector<uint8_t>* scaler_scratch) {
  if (!src_nv12 || !dst_y || !dst_uv || src_w <= 0 || src_h <= 0 ||
      dst_w <= 0 || dst_h <= 0 || dst_y_pitch < dst_w ||
      dst_uv_pitch < dst_w || (src_w & 1) != 0 || (src_h & 1) != 0 ||
      (dst_w & 1) != 0 || (dst_h & 1) != 0) {
    return false;
  }
  if (mode == EncodeMode::kPad) {
    return PadNv12EdgeClampStrided(src_nv12, src_w, src_h, dst_y, dst_y_pitch,
                                   dst_uv, dst_uv_pitch, dst_w, dst_h);
  }
  if (mode == EncodeMode::kScale) {
    const uint8_t* src_y = src_nv12;
    const uint8_t* src_uv = src_y + static_cast<size_t>(src_w) * src_h;
    return ScaleNv12ViaI420(src_y, src_w, src_uv, src_w, src_w, src_h, dst_y,
                            dst_y_pitch, dst_uv, dst_uv_pitch, dst_w, dst_h,
                            libyuv::kFilterBilinear, scaler_scratch) == 0;
  }
  if (dst_w != src_w || dst_h != src_h) {
    return false;
  }
  if (dst_y_pitch == src_w && dst_uv_pitch == src_w &&
      dst_uv == dst_y + static_cast<size_t>(src_w) * src_h) {
    std::memcpy(dst_y, src_nv12,
                static_cast<size_t>(src_w) * src_h * 3 / 2);
    return true;
  }
  for (int y = 0; y < src_h; ++y) {
    std::memcpy(dst_y + static_cast<size_t>(y) * dst_y_pitch,
                src_nv12 + static_cast<size_t>(y) * src_w,
                static_cast<size_t>(src_w));
  }
  const uint8_t* src_uv = src_nv12 + static_cast<size_t>(src_w) * src_h;
  const int uv_h = src_h / 2;
  for (int y = 0; y < uv_h; ++y) {
    std::memcpy(dst_uv + static_cast<size_t>(y) * dst_uv_pitch,
                src_uv + static_cast<size_t>(y) * src_w,
                static_cast<size_t>(src_w));
  }
  return true;
}

inline const uint8_t* ResolveCpuNv12(const RawFrame& raw,
                                     std::vector<uint8_t>& scratch) {
  if (raw.Buffer()) return raw.Buffer();
  const MiniRtcNativeVideoFrame* native = raw.NativeFrame();
  if (!native) return nullptr;
  if (native->type == MiniRtcNativeVideoFrameCpuNv12) {
    const auto& cpu = native->payload.cpu_nv12;
    if (cpu.y_plane && cpu.uv_plane && cpu.y_stride == raw.Width() &&
        cpu.uv_stride == raw.Width() &&
        cpu.uv_plane ==
            cpu.y_plane + static_cast<size_t>(raw.Width()) * raw.Height()) {
      return cpu.y_plane;
    }
  }
  size_t raw_size = 0;
  if (!GetNv12FrameSize(raw.Width(), raw.Height(), &raw_size)) {
    return nullptr;
  }
  scratch.resize(raw_size);
  if (native->copy_to_nv12(native->owner, scratch.data(), raw_size) != 0) {
    return nullptr;
  }
  return scratch.data();
}

struct Av1ObuSpan {
  size_t offset = 0;
  size_t size = 0;
  size_t payload_offset = 0;
  size_t payload_size = 0;
  uint8_t type = 0;
};

inline bool ReadLeb128(const uint8_t* data, size_t size, size_t& pos,
                       uint64_t& value) {
  value = 0;
  for (int i = 0; i < 8; ++i) {
    if (pos >= size) return false;
    const uint8_t byte = data[pos++];
    value |= static_cast<uint64_t>(byte & 0x7F) << (i * 7);
    if ((byte & 0x80) == 0) return true;
  }
  return false;
}

inline std::vector<Av1ObuSpan> ParseAv1Obus(const uint8_t* data, size_t size) {
  std::vector<Av1ObuSpan> obus;
  size_t pos = 0;
  while (pos < size) {
    const size_t start = pos;
    const uint8_t header = data[pos++];
    if ((header & 0x80) != 0) {
      // obu_forbidden_bit must be 0 in a valid raw OBU stream.
      return {};
    }
    const uint8_t type = (header >> 3) & 0x0F;
    const bool has_extension = (header & 0x04) != 0;
    const bool has_size = (header & 0x02) != 0;
    if (has_extension) {
      if (pos >= size) return {};
      ++pos;
    }
    uint64_t payload_size = 0;
    if (has_size) {
      if (!ReadLeb128(data, size, pos, payload_size) ||
          payload_size > size - pos) {
        return {};
      }
    } else {
      payload_size = size - pos;
    }
    const size_t payload_offset = pos;
    pos += static_cast<size_t>(payload_size);
    obus.push_back(Av1ObuSpan{start, pos - start, payload_offset,
                              static_cast<size_t>(payload_size), type});
  }
  return obus;
}

inline void CacheSequenceHeaderFromBytes(
    const uint8_t* data, size_t size, std::vector<uint8_t>& sequence_header) {
  for (const auto& obu : ParseAv1Obus(data, size)) {
    if (obu.type == kObuTypeSequenceHeader) {
      sequence_header.assign(data + obu.offset, data + obu.offset + obu.size);
      return;
    }
  }
}

class Av1SeqHeaderBitReader {
 public:
  Av1SeqHeaderBitReader(const uint8_t* data, size_t size)
      : data_(data), bit_size_(size * 8) {}

  bool ReadBits(int count, uint32_t& out) {
    if (count < 0 || count > 32 ||
        bit_pos_ + static_cast<size_t>(count) > bit_size_) {
      return false;
    }
    uint32_t value = 0;
    for (int i = 0; i < count; ++i) {
      const size_t byte_index = bit_pos_ >> 3;
      const int bit_offset = 7 - static_cast<int>(bit_pos_ & 7);
      value = (value << 1) | ((data_[byte_index] >> bit_offset) & 1U);
      ++bit_pos_;
    }
    out = value;
    return true;
  }

  bool SkipBits(int count) {
    if (count < 0 || bit_pos_ + static_cast<size_t>(count) > bit_size_) {
      return false;
    }
    bit_pos_ += static_cast<size_t>(count);
    return true;
  }

  bool SkipUvlc() {
    int leading_zeros = 0;
    while (true) {
      uint32_t done = 0;
      if (!ReadBits(1, done)) return false;
      if (done) break;
      ++leading_zeros;
      if (leading_zeros >= 32) return false;
    }
    return leading_zeros == 0 || SkipBits(leading_zeros);
  }

 private:
  const uint8_t* data_;
  size_t bit_size_;
  size_t bit_pos_ = 0;
};

inline bool ParseAv1SequenceHeaderDimensions(const uint8_t* obu_bytes,
                                             size_t obu_size, int& out_w,
                                             int& out_h) {
  if (!obu_bytes || obu_size == 0) return false;
  const uint8_t* payload = nullptr;
  size_t payload_size = 0;
  for (const auto& obu : ParseAv1Obus(obu_bytes, obu_size)) {
    if (obu.type == kObuTypeSequenceHeader) {
      payload = obu_bytes + obu.payload_offset;
      payload_size = obu.payload_size;
      break;
    }
  }
  if (!payload || payload_size == 0) return false;

  Av1SeqHeaderBitReader br(payload, payload_size);
  uint32_t seq_profile = 0;
  uint32_t still_picture = 0;
  uint32_t reduced_still_picture_header = 0;
  if (!br.ReadBits(3, seq_profile) || !br.ReadBits(1, still_picture) ||
      !br.ReadBits(1, reduced_still_picture_header)) {
    return false;
  }
  if (reduced_still_picture_header) {
    if (!br.SkipBits(5)) return false;
  } else {
    uint32_t timing_info_present = 0;
    uint32_t decoder_model_info_present = 0;
    uint32_t buffer_delay_length_minus_1 = 0;
    if (!br.ReadBits(1, timing_info_present)) return false;
    if (timing_info_present) {
      uint32_t equal_picture_interval = 0;
      if (!br.SkipBits(64) || !br.ReadBits(1, equal_picture_interval)) {
        return false;
      }
      if (equal_picture_interval && !br.SkipUvlc()) return false;
      if (!br.ReadBits(1, decoder_model_info_present)) return false;
      if (decoder_model_info_present) {
        if (!br.ReadBits(5, buffer_delay_length_minus_1) || !br.SkipBits(42)) {
          return false;
        }
      }
    }
    uint32_t initial_display_delay_present = 0;
    uint32_t operating_points_cnt_minus_1 = 0;
    if (!br.ReadBits(1, initial_display_delay_present) ||
        !br.ReadBits(5, operating_points_cnt_minus_1)) {
      return false;
    }
    for (uint32_t i = 0; i <= operating_points_cnt_minus_1; ++i) {
      uint32_t level_idx = 0;
      if (!br.SkipBits(12) || !br.ReadBits(5, level_idx)) return false;
      if (level_idx > 7 && !br.SkipBits(1)) return false;
      if (decoder_model_info_present) {
        uint32_t dm_op = 0;
        if (!br.ReadBits(1, dm_op)) return false;
        if (dm_op &&
            !br.SkipBits(2 * (static_cast<int>(buffer_delay_length_minus_1) + 1) +
                         1)) {
          return false;
        }
      }
      if (initial_display_delay_present) {
        uint32_t idd_op = 0;
        if (!br.ReadBits(1, idd_op)) return false;
        if (idd_op && !br.SkipBits(4)) return false;
      }
    }
  }
  uint32_t frame_width_bits_minus_1 = 0;
  uint32_t frame_height_bits_minus_1 = 0;
  uint32_t max_frame_width_minus_1 = 0;
  uint32_t max_frame_height_minus_1 = 0;
  if (!br.ReadBits(4, frame_width_bits_minus_1) ||
      !br.ReadBits(4, frame_height_bits_minus_1) ||
      !br.ReadBits(static_cast<int>(frame_width_bits_minus_1) + 1,
                   max_frame_width_minus_1) ||
      !br.ReadBits(static_cast<int>(frame_height_bits_minus_1) + 1,
                   max_frame_height_minus_1)) {
    return false;
  }
  out_w = static_cast<int>(max_frame_width_minus_1 + 1);
  out_h = static_cast<int>(max_frame_height_minus_1 + 1);
  return out_w > 0 && out_h > 0;
}

struct BitstreamProcessResult {
  bool valid = false;
  bool is_keyframe = false;
};

inline BitstreamProcessResult ProcessEncodedAv1Bitstream(
    std::vector<uint8_t>& bytes, std::vector<uint8_t>& sequence_header,
    bool hint_key, EncodeMode encode_mode, int source_width,
    int source_height,
    const std::function<void(std::vector<uint8_t>&)>& fallback_seq_lookup,
    const char* encoder_tag) {
  if (bytes.empty()) return {};
  const auto obus = ParseAv1Obus(bytes.data(), bytes.size());
  if (obus.empty()) {
    LOG_WARN("{} output is not a valid OBU stream (size={})", encoder_tag,
             bytes.size());
    return {};
  }

  bool has_seq_header = false;
  bool obu_is_key = false;
  size_t insert_offset = 0;
  size_t after_seq_offset = 0;
  for (const auto& obu : obus) {
    if (obu.type == kObuTypeTemporalDelimiter && obu.offset == 0) {
      insert_offset = obu.offset + obu.size;
      after_seq_offset = insert_offset;
    } else if (obu.type == kObuTypeSequenceHeader) {
      has_seq_header = true;
      after_seq_offset = obu.offset + obu.size;
      sequence_header.assign(bytes.data() + obu.offset,
                             bytes.data() + obu.offset + obu.size);
    } else if ((obu.type == kObuTypeFrameHeader ||
                obu.type == kObuTypeFrame) &&
               obu.payload_size > 0) {
      // In AV1 uncompressed_header (non-reduced_still_picture_header):
      // bit 7 = show_existing_frame (0), bits 6..5 = frame_type (00 = KEY).
      const uint8_t first = bytes[obu.payload_offset];
      if ((first & 0xE0) == 0x00) {
        obu_is_key = true;
      }
    }
  }

  const bool key = hint_key || has_seq_header || obu_is_key;
  if (key && !has_seq_header) {
    if (sequence_header.empty() && fallback_seq_lookup) {
      fallback_seq_lookup(sequence_header);
    }
    if (!sequence_header.empty()) {
      bytes.insert(bytes.begin() + static_cast<ptrdiff_t>(insert_offset),
                   sequence_header.begin(), sequence_header.end());
      after_seq_offset = insert_offset + sequence_header.size();
    }
  }
  int coded_w = 0;
  int coded_h = 0;
  const bool coded_larger =
      !sequence_header.empty() &&
      ParseAv1SequenceHeaderDimensions(sequence_header.data(),
                                       sequence_header.size(), coded_w,
                                       coded_h) &&
      (coded_w > source_width || coded_h > source_height);
  const bool needs_crop = (encode_mode != EncodeMode::kScale) &&
                          (encode_mode == EncodeMode::kPad || coded_larger);
  if (key && needs_crop && source_width > 0 && source_height > 0 &&
      source_width <= 65535 && source_height <= 65535) {
    av1_crop::AppendCropMetadataObu(bytes, after_seq_offset,
                                    static_cast<uint16_t>(source_width),
                                    static_cast<uint16_t>(source_height));
  }
  return {true, key};
}

template <typename CapturedTsT, typename EncodedTsT>
inline void PushEncodedAv1Frame(std::vector<uint8_t>& bytes, uint32_t out_w,
                                uint32_t out_h, bool key,
                                CapturedTsT captured_ts, EncodedTsT encoded_ts,
                                std::vector<EncodedFrame>& frames) {
  EncodedFrame frame(bytes.data(), bytes.size(), out_w, out_h);
  frame.SetEncodedWidth(out_w);
  frame.SetEncodedHeight(out_h);
  frame.SetFrameType(key ? kVideoFrameKey : kVideoFrameDelta);
  frame.SetCapturedTimestamp(captured_ts);
  frame.SetEncodedTimestamp(encoded_ts);
  frames.push_back(std::move(frame));
}

class EncodeStatsTracker {
 public:
  void Reset() {
    window_start_ = {};
    total_ms_.clear();
    wait_ms_.clear();
    input_ms_.clear();
    hw_ms_.clear();
    total_bytes_ = 0;
    keyframes_ = 0;
  }

  void Record(const char* encoder_tag, std::chrono::steady_clock::time_point now,
              int encode_width, int encode_height, bool using_d3d11,
              double total_ms, double wait_ms, double input_ms, double hw_ms,
              size_t bytes, bool keyframe) {
    if (window_start_.time_since_epoch().count() == 0) {
      window_start_ = now;
    }
    total_ms_.push_back(total_ms);
    wait_ms_.push_back(wait_ms);
    input_ms_.push_back(input_ms);
    hw_ms_.push_back(hw_ms);
    total_bytes_ += bytes;
    if (keyframe) ++keyframes_;
    const double elapsed_s =
        std::chrono::duration<double>(now - window_start_).count();
    if (elapsed_s < 5.0 || total_ms_.empty()) return;

    const size_t n = total_ms_.size();
    const double enc_p50 = Percentile(total_ms_, 0.50);
    const double enc_p95 = Percentile(total_ms_, 0.95);
    const double enc_max =
        *std::max_element(total_ms_.begin(), total_ms_.end());
    const double wait_p50 = Percentile(wait_ms_, 0.50);
    const double wait_p95 = Percentile(wait_ms_, 0.95);
    const double wait_max =
        *std::max_element(wait_ms_.begin(), wait_ms_.end());
    const double in_p50 = Percentile(input_ms_, 0.50);
    const double in_p95 = Percentile(input_ms_, 0.95);
    const double in_max =
        *std::max_element(input_ms_.begin(), input_ms_.end());
    const double hw_p50 = Percentile(hw_ms_, 0.50);
    const double hw_p95 = Percentile(hw_ms_, 0.95);
    const double hw_max =
        *std::max_element(hw_ms_.begin(), hw_ms_.end());
    const double avg_kb = static_cast<double>(total_bytes_) /
                          (static_cast<double>(n) * 1024.0);
    LOG_INFO(
        "{} stats ({:.1f}s): {}x{} d3d11={} frames={} fps={:.1f} kf={} "
        "enc_ms[p50={:.2f} p95={:.2f} max={:.2f}] "
        "wait_ms[p50={:.2f} p95={:.2f} max={:.2f}] "
        "input_ms[p50={:.2f} p95={:.2f} max={:.2f}] "
        "hw_ms[p50={:.2f} p95={:.2f} max={:.2f}] avg_kb={:.1f}",
        encoder_tag, elapsed_s, encode_width, encode_height, using_d3d11, n,
        static_cast<double>(n) / elapsed_s, keyframes_, enc_p50, enc_p95,
        enc_max, wait_p50, wait_p95, wait_max, in_p50, in_p95, in_max, hw_p50,
        hw_p95, hw_max, avg_kb);
    window_start_ = now;
    total_ms_.clear();
    wait_ms_.clear();
    input_ms_.clear();
    hw_ms_.clear();
    total_bytes_ = 0;
    keyframes_ = 0;
  }

 private:
  static double Percentile(std::vector<double>& samples, double pct) {
    if (samples.empty()) return 0.0;
    const size_t idx = (std::min)(
        samples.size() - 1,
        static_cast<size_t>(pct * static_cast<double>(samples.size() - 1)));
    std::nth_element(samples.begin(), samples.begin() + idx, samples.end());
    return samples[idx];
  }

  std::chrono::steady_clock::time_point window_start_{};
  std::vector<double> total_ms_;
  std::vector<double> wait_ms_;
  std::vector<double> input_ms_;
  std::vector<double> hw_ms_;
  size_t total_bytes_ = 0;
  size_t keyframes_ = 0;
};

}  // namespace av1_encode
}  // namespace minirtc

#endif  // MINIRTC_MEDIA_VIDEO_ENCODE_AV1_ENCODE_HELPERS_H_
