#include "video_toolbox_decoder.h"
#include <CoreMedia/CoreMedia.h>
#include <TargetConditionals.h>
#include <VideoToolbox/VideoToolbox.h>
#include <arpa/inet.h>
#include <atomic>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include "minirtc.h"
#include "video/av1_crop_metadata.h"

// #define SAVE_DECODED_NV12_STREAM
// #define SAVE_RECEIVED_H264_STREAM

namespace minirtc {
namespace {

void RetainPixelBufferOwner(void* owner) {
  if (owner) {
    CVPixelBufferRetain(static_cast<CVPixelBufferRef>(owner));
  }
}

void ReleasePixelBufferOwner(void* owner) {
  if (owner) {
    CVPixelBufferRelease(static_cast<CVPixelBufferRef>(owner));
  }
}

int CopyPixelBufferToNv12(void* owner, uint8_t* destination,
                          size_t destination_size) {
  auto pixel_buffer = static_cast<CVPixelBufferRef>(owner);
  const OSType pixel_format =
      pixel_buffer ? CVPixelBufferGetPixelFormatType(pixel_buffer) : 0;
  if (!pixel_buffer || !destination ||
      !CVPixelBufferIsPlanar(pixel_buffer) ||
      CVPixelBufferGetPlaneCount(pixel_buffer) < 2 ||
      (pixel_format != kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange &&
       pixel_format != kCVPixelFormatType_420YpCbCr8BiPlanarFullRange)) {
    return -1;
  }
  const size_t width = CVPixelBufferGetWidth(pixel_buffer);
  const size_t height = CVPixelBufferGetHeight(pixel_buffer);
  const size_t required_size = width * height * 3U / 2U;
  if (destination_size < required_size ||
      CVPixelBufferLockBaseAddress(pixel_buffer,
                                   kCVPixelBufferLock_ReadOnly) !=
          kCVReturnSuccess) {
    return -1;
  }

  const auto* y_plane = static_cast<const uint8_t*>(
      CVPixelBufferGetBaseAddressOfPlane(pixel_buffer, 0));
  const auto* uv_plane = static_cast<const uint8_t*>(
      CVPixelBufferGetBaseAddressOfPlane(pixel_buffer, 1));
  const size_t y_stride = CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer, 0);
  const size_t uv_stride = CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer, 1);
  int result = 0;
  if (!y_plane || !uv_plane || y_stride < width || uv_stride < width) {
    result = -1;
  } else {
    for (size_t row = 0; row < height; ++row) {
      memcpy(destination + row * width, y_plane + row * y_stride, width);
    }
    uint8_t* uv_destination = destination + width * height;
    for (size_t row = 0; row < height / 2U; ++row) {
      memcpy(uv_destination + row * width, uv_plane + row * uv_stride, width);
    }
  }
  CVPixelBufferUnlockBaseAddress(pixel_buffer, kCVPixelBufferLock_ReadOnly);
  return result;
}

}  // namespace

struct NaluUnit {
  const uint8_t* data;
  size_t size;
  uint8_t type;
};

std::vector<NaluUnit> ExtractNalUnits(const uint8_t* buffer, size_t size) {
  std::vector<NaluUnit> nalus;

  size_t i = 0;
  while (i + 3 <= size) {  // need at least a 3-byte start code
    size_t start_code_len = 0;
    if (buffer[i] == 0x00 && buffer[i + 1] == 0x00) {
      if (i + 3 <= size && buffer[i + 2] == 0x01) {
        start_code_len = 3;
      } else if (i + 4 <= size && buffer[i + 2] == 0x00 && buffer[i + 3] == 0x01) {
        start_code_len = 4;
      }
    }

    if (start_code_len == 0) {
      ++i;
      continue;
    }

    size_t nalu_start = i + start_code_len;
    if (nalu_start >= size) {
      break;  // no more data
    }

    // find the next start code
    size_t next_start = nalu_start + 1;  // start searching after the NALU data
    while (next_start + 2 < size) {
      if (buffer[next_start] == 0x00 && buffer[next_start + 1] == 0x00 &&
          (buffer[next_start + 2] == 0x01 ||
           (next_start + 3 < size && buffer[next_start + 2] == 0x00 && buffer[next_start + 3] == 0x01))) {
        break;
      }
      ++next_start;
    }
    
    // if no next start code found, this is the last NALU; take until buffer end
    if (next_start + 2 >= size) {
      next_start = size;
    }

    size_t nalu_size = next_start - nalu_start;
    if (nalu_size > 0) {
      uint8_t type = buffer[nalu_start] & 0x1F;
      nalus.push_back(NaluUnit{buffer + nalu_start, nalu_size, type});
    }

    i = next_start;
  }

  return nalus;
}

bool ExtractSpsPps(const uint8_t* buffer, size_t size, std::vector<uint8_t>& sps,
                   std::vector<uint8_t>& pps) {
  auto nalus = ExtractNalUnits(buffer, size);
  for (const auto& nalu : nalus) {
    if (nalu.type == 7) {
      sps.assign(nalu.data, nalu.data + nalu.size);
    } else if (nalu.type == 8) {
      pps.assign(nalu.data, nalu.data + nalu.size);
    }
  }

  return !sps.empty() && !pps.empty();
}

std::vector<NaluUnit> ExtractVideoNalUnits(const uint8_t* buffer, size_t size) {
  auto all_nalus = ExtractNalUnits(buffer, size);
  std::vector<NaluUnit> filtered;
  for (const auto& nalu : all_nalus) {
    if (nalu.type != 7 && nalu.type != 8) {
      filtered.push_back(nalu);
    }
  }
  return filtered;
}

std::vector<uint8_t> ConvertAnnexBToAVCCFiltered(const uint8_t* data, size_t size) {
  std::vector<uint8_t> avcc_data;
  auto nalus = ExtractVideoNalUnits(data, size);
  for (const auto& nalu : nalus) {
    uint32_t len = htonl(static_cast<uint32_t>(nalu.size));
    avcc_data.insert(avcc_data.end(), reinterpret_cast<uint8_t*>(&len),
                     reinterpret_cast<uint8_t*>(&len) + 4);
    avcc_data.insert(avcc_data.end(), nalu.data, nalu.data + nalu.size);
  }
  return avcc_data;
}


namespace {

class Av1BitReader {
 public:
  Av1BitReader(const uint8_t* data, size_t size)
      : data_(data), bit_size_(size * 8) {}

  bool ReadBits(int count, uint32_t& out) {
    if (count < 0 || count > 32 || bit_pos_ + static_cast<size_t>(count) > bit_size_) {
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
    if (leading_zeros > 0 && !SkipBits(leading_zeros)) return false;
    return true;
  }

 private:
  const uint8_t* data_;
  size_t bit_size_;
  size_t bit_pos_ = 0;
};

bool ExtractAv1SequenceHeaderObu(const uint8_t* data, size_t size,
                                 std::vector<uint8_t>& seq_obu_raw,
                                 const uint8_t*& seq_payload,
                                 size_t& seq_payload_size) {
  size_t offset = 0;
  while (offset < size) {
    const size_t obu_start = offset;
    const uint8_t header = data[offset++];
    if ((header & 0x80) != 0) {
      return false;
    }
    const uint8_t obu_type = (header >> 3) & 0x0F;
    const bool has_extension = (header & 0x04) != 0;
    const bool has_size = (header & 0x02) != 0;

    if (has_extension) {
      if (offset >= size) return false;
      ++offset;
    }

    size_t obu_size = 0;
    if (has_size) {
      uint64_t leb = 0;
      int shift = 0;
      bool finished = false;
      for (int i = 0; i < 8 && offset < size; ++i) {
        const uint8_t byte = data[offset++];
        leb |= static_cast<uint64_t>(byte & 0x7F) << shift;
        if ((byte & 0x80) == 0) {
          finished = true;
          break;
        }
        shift += 7;
      }
      if (!finished || leb > size - offset) return false;
      obu_size = static_cast<size_t>(leb);
    } else {
      obu_size = size - offset;
    }

    const uint8_t* payload_ptr = data + offset;
    offset += obu_size;

    if (obu_type == 1) {  // OBU_SEQUENCE_HEADER
      seq_obu_raw.assign(data + obu_start, data + offset);
      seq_payload = payload_ptr;
      seq_payload_size = obu_size;
      return true;
    }
  }
  return false;
}

bool ParseAv1SequenceHeaderAndBuildAv1C(const std::vector<uint8_t>& seq_obu_raw,
                                        const uint8_t* seq_payload,
                                        size_t seq_payload_size,
                                        int32_t& width, int32_t& height,
                                        std::vector<uint8_t>& av1c) {
  Av1BitReader br(seq_payload, seq_payload_size);
  uint32_t seq_profile = 0;
  uint32_t still_picture = 0;
  uint32_t reduced_still_picture_header = 0;
  if (!br.ReadBits(3, seq_profile) || !br.ReadBits(1, still_picture) ||
      !br.ReadBits(1, reduced_still_picture_header)) {
    return false;
  }

  uint32_t seq_level_idx_0 = 0;
  uint32_t seq_tier_0 = 0;
  if (reduced_still_picture_header) {
    if (!br.ReadBits(5, seq_level_idx_0)) return false;
  } else {
    uint32_t timing_info_present = 0;
    uint32_t decoder_model_info_present = 0;
    uint32_t buffer_delay_length_minus_1 = 0;
    if (!br.ReadBits(1, timing_info_present)) return false;
    if (timing_info_present) {
      uint32_t equal_picture_interval = 0;
      if (!br.SkipBits(32) || !br.SkipBits(32) ||
          !br.ReadBits(1, equal_picture_interval)) {
        return false;
      }
      if (equal_picture_interval && !br.SkipUvlc()) return false;
      if (!br.ReadBits(1, decoder_model_info_present)) return false;
      if (decoder_model_info_present) {
        if (!br.ReadBits(5, buffer_delay_length_minus_1) ||
            !br.SkipBits(32) || !br.SkipBits(5) || !br.SkipBits(5)) {
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
      uint32_t tier = 0;
      if (!br.SkipBits(12) || !br.ReadBits(5, level_idx)) return false;
      if (level_idx > 7 && !br.ReadBits(1, tier)) return false;
      if (i == 0) {
        seq_level_idx_0 = level_idx;
        seq_tier_0 = tier;
      }
      if (decoder_model_info_present) {
        uint32_t decoder_model_present_for_op = 0;
        if (!br.ReadBits(1, decoder_model_present_for_op)) return false;
        if (decoder_model_present_for_op) {
          const int n = static_cast<int>(buffer_delay_length_minus_1) + 1;
          if (!br.SkipBits(n) || !br.SkipBits(n) || !br.SkipBits(1)) {
            return false;
          }
        }
      }
      if (initial_display_delay_present) {
        uint32_t initial_display_delay_present_for_op = 0;
        if (!br.ReadBits(1, initial_display_delay_present_for_op)) return false;
        if (initial_display_delay_present_for_op && !br.SkipBits(4)) {
          return false;
        }
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
  width = static_cast<int32_t>(max_frame_width_minus_1 + 1);
  height = static_cast<int32_t>(max_frame_height_minus_1 + 1);

  if (!reduced_still_picture_header) {
    uint32_t frame_id_numbers_present = 0;
    if (!br.ReadBits(1, frame_id_numbers_present)) return false;
    if (frame_id_numbers_present && !br.SkipBits(7)) return false;
  }

  if (!br.SkipBits(3)) return false;

  if (!reduced_still_picture_header) {
    uint32_t enable_order_hint = 0;
    if (!br.SkipBits(4) || !br.ReadBits(1, enable_order_hint)) return false;
    if (enable_order_hint && !br.SkipBits(2)) return false;
    uint32_t seq_choose_screen_content_tools = 0;
    if (!br.ReadBits(1, seq_choose_screen_content_tools)) return false;
    uint32_t seq_force_screen_content_tools = 2;
    if (!seq_choose_screen_content_tools &&
        !br.ReadBits(1, seq_force_screen_content_tools)) {
      return false;
    }
    if (seq_force_screen_content_tools > 0) {
      uint32_t seq_choose_integer_mv = 0;
      if (!br.ReadBits(1, seq_choose_integer_mv)) return false;
      if (!seq_choose_integer_mv && !br.SkipBits(1)) return false;
    }
    if (enable_order_hint && !br.SkipBits(3)) return false;
  }

  if (!br.SkipBits(3)) return false;

  uint32_t high_bitdepth = 0;
  uint32_t twelve_bit = 0;
  uint32_t mono_chrome = 0;
  uint32_t color_description_present = 0;
  uint32_t color_primaries = 2;
  uint32_t transfer_characteristics = 2;
  uint32_t matrix_coefficients = 2;
  if (!br.ReadBits(1, high_bitdepth)) return false;
  if (seq_profile == 2 && high_bitdepth && !br.ReadBits(1, twelve_bit)) {
    return false;
  }
  if (seq_profile != 1 && !br.ReadBits(1, mono_chrome)) return false;
  if (!br.ReadBits(1, color_description_present)) return false;
  if (color_description_present) {
    if (!br.ReadBits(8, color_primaries) ||
        !br.ReadBits(8, transfer_characteristics) ||
        !br.ReadBits(8, matrix_coefficients)) {
      return false;
    }
  }

  uint32_t subsampling_x = 1;
  uint32_t subsampling_y = 1;
  uint32_t chroma_sample_position = 0;
  if (mono_chrome) {
    if (!br.SkipBits(1)) return false;
  } else if (color_primaries == 1 && transfer_characteristics == 13 &&
             matrix_coefficients == 0) {
    subsampling_x = 0;
    subsampling_y = 0;
  } else {
    if (!br.SkipBits(1)) return false;
    if (seq_profile == 0) {
      subsampling_x = 1;
      subsampling_y = 1;
    } else if (seq_profile == 1) {
      subsampling_x = 0;
      subsampling_y = 0;
    } else {
      if (twelve_bit) {
        if (!br.ReadBits(1, subsampling_x)) return false;
        if (subsampling_x) {
          if (!br.ReadBits(1, subsampling_y)) return false;
        } else {
          subsampling_y = 0;
        }
      } else {
        subsampling_x = 1;
        subsampling_y = 0;
      }
    }
    if (subsampling_x && subsampling_y &&
        !br.ReadBits(2, chroma_sample_position)) {
      return false;
    }
  }

  av1c.clear();
  av1c.reserve(4 + seq_obu_raw.size());
  av1c.push_back(0x81);
  av1c.push_back(static_cast<uint8_t>(((seq_profile & 0x07) << 5) |
                                      (seq_level_idx_0 & 0x1F)));
  av1c.push_back(static_cast<uint8_t>(
      ((seq_tier_0 & 0x01) << 7) | ((high_bitdepth & 0x01) << 6) |
      ((twelve_bit & 0x01) << 5) | ((mono_chrome & 0x01) << 4) |
      ((subsampling_x & 0x01) << 3) | ((subsampling_y & 0x01) << 2) |
      (chroma_sample_position & 0x03)));
  av1c.push_back(0x00);
  av1c.insert(av1c.end(), seq_obu_raw.begin(), seq_obu_raw.end());
  return width > 0 && height > 0;
}

}  // namespace

class VideoToolboxDecoder::Impl {
 public:
  Impl(std::shared_ptr<SystemClock> clock, bool native_video_output,
       VideoCodecType codec_type);
  ~Impl();

  int Init();
  int Decode(std::unique_ptr<ReceivedFrame> frame,
             std::function<void(const DecodedFrame*)> on_decoded_cb);
  std::string GetDecoderName() const;

 private:
  bool CreateSession(const std::vector<uint8_t>& sps, const std::vector<uint8_t>& pps);
  bool CreateAv1Session(const std::vector<uint8_t>& seq_obu_raw,
                        const uint8_t* seq_payload, size_t seq_payload_size);
  bool CreateDecompressionSessionFromFormatDesc();
  CVPixelBufferRef CreateCroppedPixelBuffer(CVPixelBufferRef src, uint32_t out_w,
                                            uint32_t out_h, OSType fmt);
  static void DecodeCallback(void* decompression_output_ref_con, void* source_frame_ref_con,
                             OSStatus status, VTDecodeInfoFlags info_flags,
                             CVImageBufferRef image_buffer, CMTime pts, CMTime duration);

 private:
  std::shared_ptr<SystemClock> clock_;
  const bool native_video_output_;
  const VideoCodecType codec_type_;
  std::function<void(const DecodedFrame*)> on_receive_decoded_frame_;
  std::atomic<uint64_t> submitted_frame_count_{0};
  std::atomic<uint64_t> decoded_frame_count_{0};
  std::atomic<bool> session_needs_recovery_{false};
  struct FrameTimestamps {
    int64_t captured_us;
    int64_t received_us;
    uint32_t crop_width;
    uint32_t crop_height;
  };
  std::mutex timestamps_mutex_;
  std::unordered_map<uintptr_t, FrameTimestamps> pending_timestamps_;
  uintptr_t next_frame_id_ = 0;

  VTDecompressionSessionRef decompression_session_;
  CMVideoFormatDescriptionRef format_desc_;
  std::vector<uint8_t> last_sps_;
  std::vector<uint8_t> last_pps_;
  std::vector<uint8_t> last_av1_seq_obu_;
  uint32_t av1_crop_width_ = 0;
  uint32_t av1_crop_height_ = 0;
  CVPixelBufferPoolRef cropped_pool_ = nullptr;
  uint32_t cropped_pool_width_ = 0;
  uint32_t cropped_pool_height_ = 0;
  OSType cropped_pool_format_ = 0;

  FILE* file_nv12_ = nullptr;
  FILE* file_h264_ = nullptr;
  std::string h264_file_name_;
  std::string nv12_file_name_;
};

VideoToolboxDecoder::Impl::Impl(std::shared_ptr<SystemClock> clock,
                               bool native_video_output,
                               VideoCodecType codec_type)
    : clock_(std::move(clock)),
      native_video_output_(native_video_output),
      codec_type_(codec_type),
      decompression_session_(nullptr),
      format_desc_(nullptr) {}

VideoToolboxDecoder::Impl::~Impl() {
  if (decompression_session_) {
    VTDecompressionSessionWaitForAsynchronousFrames(decompression_session_);
    VTDecompressionSessionInvalidate(decompression_session_);
    CFRelease(decompression_session_);
  }
  if (format_desc_) {
    CFRelease(format_desc_);
  }
  if (cropped_pool_) {
    CVPixelBufferPoolRelease(cropped_pool_);
    cropped_pool_ = nullptr;
  }
  last_sps_.clear();
  last_pps_.clear();
  last_av1_seq_obu_.clear();

#ifdef SAVE_DECODED_NV12_STREAM
  if (file_nv12_) {
    fflush(file_nv12_);
    file_nv12_ = nullptr;
  }
#endif

#ifdef SAVE_RECEIVED_H264_STREAM
  if (file_h264_) {
    fflush(file_h264_);
    file_h264_ = nullptr;
  }
#endif
}

int VideoToolboxDecoder::Impl::Init() {
#if defined(SAVE_DECODED_NV12_STREAM) || defined(SAVE_RECEIVED_H264_STREAM)
#if defined(MINIRTC_IOS)
  std::string log_dir = std::string(getenv("TMPDIR")) + "/CrossDesk/";
#else
  std::string log_dir = std::string(getenv("HOME")) + "/Library/Logs/CrossDesk/";
  // Create directory if not exists
  std::string mkdir_cmd = "mkdir -p " + log_dir;
  system(mkdir_cmd.c_str());
#endif
#endif

#ifdef SAVE_DECODED_NV12_STREAM
  nv12_file_name_ =
      log_dir + "decoded_nv12_stream_" + std::to_string(reinterpret_cast<uintptr_t>(this)) + ".yuv";
  file_nv12_ = fopen(nv12_file_name_.c_str(), "w+b");
  if (!file_nv12_) {
    LOG_WARN("Fail to open {}", nv12_file_name_.c_str());
  }
#endif

#ifdef SAVE_RECEIVED_H264_STREAM
  h264_file_name_ =
      log_dir + "received_h264_stream_" + std::to_string(reinterpret_cast<uintptr_t>(this)) + ".h264";
  file_h264_ = fopen(h264_file_name_.c_str(), "w+b");
  if (!file_h264_) {
    LOG_WARN("Fail to open {}", h264_file_name_.c_str());
  }
#endif
  return 0;
}

int VideoToolboxDecoder::Impl::Decode(
    std::unique_ptr<ReceivedFrame> received_frame,
    std::function<void(const DecodedFrame*)> on_receive_decoded_frame) {
  if (!received_frame) {
    LOG_ERROR("Received frame is null");
    return -1;
  }

  if (!on_receive_decoded_frame_) {
    on_receive_decoded_frame_ = on_receive_decoded_frame;
  }

  const uint8_t* data = received_frame->Buffer();
  size_t size = received_frame->Size();

#ifdef SAVE_RECEIVED_H264_STREAM
  if (file_h264_) {
    fwrite((unsigned char*)data, 1, size, file_h264_);
  }
#endif

  bool is_keyframe = false;
  std::vector<uint8_t> avcc_data;
  std::vector<uint8_t> stripped_av1;
  const uint8_t* sample_bytes = nullptr;
  size_t sample_size = 0;

  if (codec_type_ == VideoCodecType::AV1) {
    uint32_t meta_w = 0;
    uint32_t meta_h = 0;
    bool meta_has_seq = false;
    av1_crop::ExtractAndStripCropMetadata(data, size, meta_w, meta_h,
                                          meta_has_seq, stripped_av1);
    if (meta_has_seq) {
      av1_crop_width_ = meta_w;
      av1_crop_height_ = meta_h;
    }
    const uint8_t* av1_data =
        stripped_av1.empty() ? data : stripped_av1.data();
    const size_t av1_size = stripped_av1.empty() ? size : stripped_av1.size();

    std::vector<uint8_t> seq_obu_raw;
    const uint8_t* seq_payload = nullptr;
    size_t seq_payload_size = 0;
    const bool has_seq_hdr = ExtractAv1SequenceHeaderObu(
        av1_data, av1_size, seq_obu_raw, seq_payload, seq_payload_size);
    if (has_seq_hdr) {
      is_keyframe = true;
      if (seq_obu_raw != last_av1_seq_obu_) {
        last_av1_seq_obu_ = seq_obu_raw;
        session_needs_recovery_.store(true);
      }
    }

    if (!decompression_session_ || session_needs_recovery_.load()) {
      if (!has_seq_hdr || seq_obu_raw.empty()) return -1;
      LOG_INFO("Creating/recovering VideoToolbox AV1 decompression session");
      if (!CreateAv1Session(seq_obu_raw, seq_payload, seq_payload_size)) {
        LOG_ERROR("Failed to create VideoToolbox AV1 decompression session");
        return -1;
      }
    }

    if (!av1_data || av1_size == 0) {
      LOG_ERROR("Empty AV1 frame data");
      return -1;
    }
    sample_bytes = av1_data;
    sample_size = av1_size;
  } else {
    // check if frame contains SPS (NAL type 7) using a more robust method
    // no longer assume start code is 4 bytes, parse directly with ExtractNalUnits
    bool has_sps = false;
    auto all_nalus = ExtractNalUnits(data, size);
    for (const auto& nalu : all_nalus) {
      if (nalu.type == 7) {  // SPS
        has_sps = true;
      } else if (nalu.type == 5) {  // IDR
        is_keyframe = true;
      }
    }

    if (has_sps) {
      std::vector<uint8_t> sps, pps;
      if (!ExtractSpsPps(data, size, sps, pps)) {
        LOG_ERROR("Failed to extract SPS/PPS from frame data");
        return -1;
      }

      if (sps != last_sps_ || pps != last_pps_) {
        last_sps_ = std::move(sps);
        last_pps_ = std::move(pps);
        session_needs_recovery_.store(true);
      }
    }

    if (!decompression_session_ || session_needs_recovery_.load()) {
      // A decoder invalidated by iOS backgrounding must be recreated even when
      // the stream's SPS/PPS are unchanged. Resume at an IDR so references from
      // the old session are never used by the replacement session. Returning an
      // error while waiting also makes the receiver request a key frame.
      if (!is_keyframe || last_sps_.empty() || last_pps_.empty()) return -1;
      LOG_INFO("Creating/recovering VideoToolbox decompression session at IDR");
      if (!CreateSession(last_sps_, last_pps_)) {
        LOG_ERROR("Failed to create decompression session");
        return -1;
      }
    }

    avcc_data = ConvertAnnexBToAVCCFiltered(data, size);
    if (avcc_data.empty()) {
      LOG_ERROR("Failed to convert Annex B to AVCC format");
      return -1;
    }
    sample_bytes = avcc_data.data();
    sample_size = avcc_data.size();
  }

  if (!decompression_session_) {
    LOG_ERROR("Decompression session is not initialized");
    return -1;
  }

  CMBlockBufferRef block_buffer = nullptr;
  OSStatus status = CMBlockBufferCreateWithMemoryBlock(
      kCFAllocatorDefault, nullptr, sample_size, kCFAllocatorDefault,
      nullptr, 0, sample_size, 0, &block_buffer);
  if (status != kCMBlockBufferNoErr) {
    LOG_ERROR("Failed to create block buffer, status: {}", status);
    return -1;
  }
  status = CMBlockBufferReplaceDataBytes(sample_bytes, block_buffer, 0,
                                         sample_size);
  if (status != kCMBlockBufferNoErr) {
    LOG_ERROR("Failed to copy frame data into block buffer, status: {}", status);
    CFRelease(block_buffer);
    return -1;
  }

  CMSampleBufferRef sample_buffer = nullptr;
  CMSampleTimingInfo timing = {};
  timing.duration = kCMTimeInvalid;
  timing.presentationTimeStamp = CMTimeMake(received_frame->ReceivedTimestamp(), 1'000'000);
  timing.decodeTimeStamp = kCMTimeInvalid;
  // A compressed video access unit is one sample. Supplying zero sample-size
  // entries creates a formally valid CMSampleBuffer whose sample size can be
  // reported as zero, which VideoToolbox accepts without producing output.
  status = CMSampleBufferCreateReady(nullptr, block_buffer, format_desc_, 1, 1,
                                     &timing, 1, &sample_size, &sample_buffer);
  CFRelease(block_buffer);
  if (status != noErr) {
    LOG_ERROR("Failed to create sample buffer, status: {}", status);
    return -1;
  }

  CFArrayRef attachments =
      CMSampleBufferGetSampleAttachmentsArray(sample_buffer, true);
  if (attachments && CFArrayGetCount(attachments) > 0) {
    auto attachment = static_cast<CFMutableDictionaryRef>(
        const_cast<void*>(CFArrayGetValueAtIndex(attachments, 0)));
    CFDictionarySetValue(attachment, kCMSampleAttachmentKey_DisplayImmediately,
                         kCFBooleanTrue);
    CFDictionarySetValue(attachment, kCMSampleAttachmentKey_NotSync,
                         is_keyframe ? kCFBooleanFalse : kCFBooleanTrue);
  }

  uintptr_t frame_id;
  {
    std::lock_guard lock(timestamps_mutex_);
    frame_id = ++next_frame_id_;
    pending_timestamps_[frame_id] = {
        received_frame->CapturedTimestamp(),
        received_frame->ReceivedTimestamp(),
        codec_type_ == VideoCodecType::AV1 ? av1_crop_width_ : 0,
        codec_type_ == VideoCodecType::AV1 ? av1_crop_height_ : 0};
  }
  VTDecodeInfoFlags decode_flags = 0;
  status = VTDecompressionSessionDecodeFrame(decompression_session_, sample_buffer,
                                             kVTDecodeFrame_EnableAsynchronousDecompression,
                                             reinterpret_cast<void*>(frame_id), &decode_flags);
  if (status != noErr || (decode_flags & kVTDecodeInfo_FrameDropped)) {
    std::lock_guard lock(timestamps_mutex_);
    pending_timestamps_.erase(frame_id);
  }
  CFRelease(sample_buffer);
  if (status != noErr) {
    session_needs_recovery_.store(true);
    LOG_ERROR("VTDecompressionSessionDecodeFrame failed, status: {}", status);
  } else {
    const uint64_t frame_count = ++submitted_frame_count_;
    if (frame_count == 1 || frame_count % 300 == 0) {
      LOG_DEBUG("VideoToolbox submitted frame {}, sample_size={}, keyframe={}",
               frame_count, sample_size, is_keyframe);
    }
  }
  return (status == noErr) ? 0 : -1;
}

std::string VideoToolboxDecoder::Impl::GetDecoderName() const {
  return codec_type_ == VideoCodecType::AV1 ? "VideoToolbox (AV1)"
                                            : "VideoToolbox (H.264)";
}

bool VideoToolboxDecoder::Impl::CreateSession(const std::vector<uint8_t>& sps,
                                              const std::vector<uint8_t>& pps) {
  if (decompression_session_) {
    VTDecompressionSessionWaitForAsynchronousFrames(decompression_session_);
    VTDecompressionSessionInvalidate(decompression_session_);
    CFRelease(decompression_session_);
    decompression_session_ = nullptr;
  }
  {
    std::lock_guard lock(timestamps_mutex_);
    pending_timestamps_.clear();
  }
  if (format_desc_) {
    CFRelease(format_desc_);
    format_desc_ = nullptr;
  }
  // WaitForAsynchronousFrames above can deliver errors from the old session.
  // Clear them only after it has drained; callbacks never recreate a session.
  session_needs_recovery_.store(false);
  const uint8_t* sets[] = {sps.data(), pps.data()};
  const size_t sizes[] = {sps.size(), pps.size()};
  size_t param_set_cnt = 2;  // at least 2 (SPS and PPS)
  size_t nalu_head_len = 4;  // 1, 2, 4 bytes are common, but 4 is safest for H.264
  OSStatus status = CMVideoFormatDescriptionCreateFromH264ParameterSets(
      nullptr, param_set_cnt, sets, sizes, nalu_head_len, &format_desc_);
  if (status != noErr) {
    LOG_ERROR("Failed to create format description from SPS/PPS");
    return false;
  }

  return CreateDecompressionSessionFromFormatDesc();
}

bool VideoToolboxDecoder::Impl::CreateAv1Session(
    const std::vector<uint8_t>& seq_obu_raw, const uint8_t* seq_payload,
    size_t seq_payload_size) {
  if (decompression_session_) {
    VTDecompressionSessionWaitForAsynchronousFrames(decompression_session_);
    VTDecompressionSessionInvalidate(decompression_session_);
    CFRelease(decompression_session_);
    decompression_session_ = nullptr;
  }
  {
    std::lock_guard lock(timestamps_mutex_);
    pending_timestamps_.clear();
  }
  if (format_desc_) {
    CFRelease(format_desc_);
    format_desc_ = nullptr;
  }
  session_needs_recovery_.store(false);

  int32_t width = 0;
  int32_t height = 0;
  std::vector<uint8_t> av1c;
  if (!ParseAv1SequenceHeaderAndBuildAv1C(seq_obu_raw, seq_payload,
                                          seq_payload_size, width, height,
                                          av1c)) {
    LOG_ERROR("Failed to parse AV1 sequence header OBU");
    return false;
  }

  CFDataRef av1c_data = CFDataCreate(
      kCFAllocatorDefault, av1c.data(), static_cast<CFIndex>(av1c.size()));
  if (!av1c_data) {
    LOG_ERROR("Failed to allocate CFData for av1C");
    return false;
  }
  const void* atom_keys[] = {CFSTR("av1C")};
  const void* atom_values[] = {av1c_data};
  CFDictionaryRef atoms = CFDictionaryCreate(
      kCFAllocatorDefault, atom_keys, atom_values, 1,
      &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  const void* ext_keys[] = {
      kCMFormatDescriptionExtension_SampleDescriptionExtensionAtoms};
  const void* ext_values[] = {atoms};
  CFDictionaryRef extensions = CFDictionaryCreate(
      kCFAllocatorDefault, ext_keys, ext_values, 1,
      &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);

  OSStatus status = CMVideoFormatDescriptionCreate(
      kCFAllocatorDefault, kCMVideoCodecType_AV1, width, height, extensions,
      &format_desc_);
  CFRelease(extensions);
  CFRelease(atoms);
  CFRelease(av1c_data);
  if (status != noErr) {
    LOG_ERROR("Failed to create AV1 format description ({}x{}), status: {}",
              width, height, status);
    return false;
  }

  LOG_INFO("Created VideoToolbox AV1 format description: {}x{}", width, height);
  return CreateDecompressionSessionFromFormatDesc();
}

bool VideoToolboxDecoder::Impl::CreateDecompressionSessionFromFormatDesc() {
  VTDecompressionOutputCallbackRecord callback = {};
  callback.decompressionOutputCallback = &DecodeCallback;
  callback.decompressionOutputRefCon = this;

  CFMutableDictionaryRef decoder_spec = CFDictionaryCreateMutable(
      kCFAllocatorDefault, 2, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  // Leave the simulator free to select its available software decoder.
#if TARGET_OS_IOS && !TARGET_OS_SIMULATOR
  if (@available(iOS 17.0, *)) {
    CFDictionarySetValue(
        decoder_spec,
        kVTVideoDecoderSpecification_EnableHardwareAcceleratedVideoDecoder,
        kCFBooleanTrue);
  }
#elif !TARGET_OS_IOS
  CFDictionarySetValue(
      decoder_spec,
      kVTVideoDecoderSpecification_EnableHardwareAcceleratedVideoDecoder,
      kCFBooleanTrue);
  if (codec_type_ == VideoCodecType::AV1) {
    CFDictionarySetValue(
        decoder_spec,
        kVTVideoDecoderSpecification_RequireHardwareAcceleratedVideoDecoder,
        kCFBooleanTrue);
  }
#endif

  CFMutableDictionaryRef output_attributes = CFDictionaryCreateMutable(
      kCFAllocatorDefault, 3, &kCFTypeDictionaryKeyCallBacks,
      &kCFTypeDictionaryValueCallBacks);
  int32_t pixel_format = kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange;
  CFNumberRef pixel_format_number = CFNumberCreate(
      kCFAllocatorDefault, kCFNumberSInt32Type, &pixel_format);
  CFDictionarySetValue(output_attributes, kCVPixelBufferPixelFormatTypeKey,
                       pixel_format_number);
  CFDictionarySetValue(output_attributes, kCVPixelBufferMetalCompatibilityKey,
                       kCFBooleanTrue);
  CFDictionaryRef io_surface_properties = CFDictionaryCreate(
      kCFAllocatorDefault, nullptr, nullptr, 0,
      &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  CFDictionarySetValue(output_attributes, kCVPixelBufferIOSurfacePropertiesKey,
                       io_surface_properties);

  OSStatus status = VTDecompressionSessionCreate(
      nullptr, format_desc_, decoder_spec, output_attributes, &callback,
      &decompression_session_);
  CFRelease(io_surface_properties);
  CFRelease(pixel_format_number);
  CFRelease(output_attributes);
  CFRelease(decoder_spec);
  if (status != noErr) {
    LOG_ERROR("Failed to create VideoToolbox decompression session, status: {}",
              status);
    return false;
  }
  const OSStatus rt_status = VTSessionSetProperty(
      decompression_session_, kVTDecompressionPropertyKey_RealTime,
      kCFBooleanTrue);
  if (rt_status != noErr) {
    LOG_WARN(
        "Failed to set VideoToolbox real-time decompression property: {}",
        rt_status);
  }
  return true;
}

CVPixelBufferRef VideoToolboxDecoder::Impl::CreateCroppedPixelBuffer(
    CVPixelBufferRef src, uint32_t out_w, uint32_t out_h, OSType fmt) {
  if (!src || out_w == 0 || out_h == 0) return nullptr;
  if (!cropped_pool_ || cropped_pool_width_ != out_w ||
      cropped_pool_height_ != out_h || cropped_pool_format_ != fmt) {
    if (cropped_pool_) {
      CVPixelBufferPoolRelease(cropped_pool_);
      cropped_pool_ = nullptr;
    }
    CFMutableDictionaryRef attrs = CFDictionaryCreateMutable(
        kCFAllocatorDefault, 5, &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);
    int32_t w_val = static_cast<int32_t>(out_w);
    int32_t h_val = static_cast<int32_t>(out_h);
    int32_t fmt_val = static_cast<int32_t>(fmt);
    CFNumberRef w_num = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &w_val);
    CFNumberRef h_num = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &h_val);
    CFNumberRef fmt_num = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &fmt_val);
    CFDictionaryRef io_props = CFDictionaryCreate(
        kCFAllocatorDefault, nullptr, nullptr, 0,
        &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    CFDictionarySetValue(attrs, kCVPixelBufferWidthKey, w_num);
    CFDictionarySetValue(attrs, kCVPixelBufferHeightKey, h_num);
    CFDictionarySetValue(attrs, kCVPixelBufferPixelFormatTypeKey, fmt_num);
    CFDictionarySetValue(attrs, kCVPixelBufferMetalCompatibilityKey, kCFBooleanTrue);
    CFDictionarySetValue(attrs, kCVPixelBufferIOSurfacePropertiesKey, io_props);
    CVReturn pool_rc = CVPixelBufferPoolCreate(kCFAllocatorDefault, nullptr,
                                               attrs, &cropped_pool_);
    CFRelease(io_props);
    CFRelease(fmt_num);
    CFRelease(h_num);
    CFRelease(w_num);
    CFRelease(attrs);
    if (pool_rc != kCVReturnSuccess || !cropped_pool_) {
      cropped_pool_ = nullptr;
      return nullptr;
    }
    cropped_pool_width_ = out_w;
    cropped_pool_height_ = out_h;
    cropped_pool_format_ = fmt;
  }

  CVPixelBufferRef dst = nullptr;
  if (CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, cropped_pool_,
                                         &dst) != kCVReturnSuccess ||
      !dst) {
    return nullptr;
  }
  if (CVPixelBufferLockBaseAddress(src, kCVPixelBufferLock_ReadOnly) !=
      kCVReturnSuccess) {
    CFRelease(dst);
    return nullptr;
  }
  if (CVPixelBufferLockBaseAddress(dst, 0) != kCVReturnSuccess) {
    CVPixelBufferUnlockBaseAddress(src, kCVPixelBufferLock_ReadOnly);
    CFRelease(dst);
    return nullptr;
  }

  const size_t src_y_stride = CVPixelBufferGetBytesPerRowOfPlane(src, 0);
  const size_t src_uv_stride = CVPixelBufferGetBytesPerRowOfPlane(src, 1);
  const size_t dst_y_stride = CVPixelBufferGetBytesPerRowOfPlane(dst, 0);
  const size_t dst_uv_stride = CVPixelBufferGetBytesPerRowOfPlane(dst, 1);
  const uint8_t* src_y =
      static_cast<const uint8_t*>(CVPixelBufferGetBaseAddressOfPlane(src, 0));
  const uint8_t* src_uv =
      static_cast<const uint8_t*>(CVPixelBufferGetBaseAddressOfPlane(src, 1));
  uint8_t* dst_y = static_cast<uint8_t*>(CVPixelBufferGetBaseAddressOfPlane(dst, 0));
  uint8_t* dst_uv = static_cast<uint8_t*>(CVPixelBufferGetBaseAddressOfPlane(dst, 1));

  bool ok = src_y && src_uv && dst_y && dst_uv && src_y_stride >= out_w &&
            dst_y_stride >= out_w && src_uv_stride >= out_w &&
            dst_uv_stride >= out_w;
  if (ok) {
    for (size_t row = 0; row < out_h; ++row) {
      memcpy(dst_y + row * dst_y_stride, src_y + row * src_y_stride, out_w);
    }
    for (size_t row = 0; row < out_h / 2; ++row) {
      memcpy(dst_uv + row * dst_uv_stride, src_uv + row * src_uv_stride, out_w);
    }
  }
  CVPixelBufferUnlockBaseAddress(dst, 0);
  CVPixelBufferUnlockBaseAddress(src, kCVPixelBufferLock_ReadOnly);
  if (!ok) {
    CFRelease(dst);
    return nullptr;
  }
  CVBufferPropagateAttachments(src, dst);
  return dst;
}

void VideoToolboxDecoder::Impl::DecodeCallback(void* decompression_output_ref_con,
                                               void* source_frame_ref_con, OSStatus status,
                                               VTDecodeInfoFlags info_flags,
                                               CVImageBufferRef image_buffer, CMTime pts,
                                               CMTime duration) {
  VideoToolboxDecoder::Impl* impl =
      static_cast<VideoToolboxDecoder::Impl*>(decompression_output_ref_con);
  if (!impl) {
    LOG_ERROR("Decode callback received null decompression output ref con");
    return;
  }

  FrameTimestamps timestamps{};
  {
    std::lock_guard lock(impl->timestamps_mutex_);
    const auto it =
        impl->pending_timestamps_.find(reinterpret_cast<uintptr_t>(source_frame_ref_con));
    if (it != impl->pending_timestamps_.end()) {
      timestamps = it->second;
      impl->pending_timestamps_.erase(it);
    }
  }

  if (status != noErr) {
    impl->session_needs_recovery_.store(true);
    LOG_ERROR("Decode callback received error status: {}", status);
    return;
  }

  if (impl->session_needs_recovery_.load()) return;

  if (!image_buffer) {
    LOG_ERROR("Decode callback received null image buffer");
    return;
  }

  CVPixelBufferRef pixel_buffer = static_cast<CVPixelBufferRef>(image_buffer);
  const OSType pixel_format = CVPixelBufferGetPixelFormatType(pixel_buffer);
  if (!CVPixelBufferIsPlanar(pixel_buffer) ||
      CVPixelBufferGetPlaneCount(pixel_buffer) < 2 ||
      (pixel_format != kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange &&
       pixel_format != kCVPixelFormatType_420YpCbCr8BiPlanarFullRange)) {
    LOG_ERROR("Unexpected VideoToolbox output pixel format: {}, planes: {}",
              pixel_format, CVPixelBufferGetPlaneCount(pixel_buffer));
    return;
  }

  const uint32_t width = static_cast<uint32_t>(CVPixelBufferGetWidth(pixel_buffer));
  const uint32_t height = static_cast<uint32_t>(CVPixelBufferGetHeight(pixel_buffer));
  uint32_t out_width = width;
  uint32_t out_height = height;
  if (timestamps.crop_width > 0 && timestamps.crop_height > 0 &&
      timestamps.crop_width <= width && timestamps.crop_height <= height &&
      (timestamps.crop_width & 1U) == 0 && (timestamps.crop_height & 1U) == 0 &&
      (timestamps.crop_width < width || timestamps.crop_height < height)) {
    out_width = timestamps.crop_width;
    out_height = timestamps.crop_height;
  }

  DecodedFrame decoded_frame;
  MiniRtcNativeVideoFrame native_frame{};
  CVPixelBufferRef cropped_buffer = nullptr;
  if (impl->native_video_output_) {
    CVPixelBufferRef present_buffer = pixel_buffer;
    if (out_width != width || out_height != height) {
      cropped_buffer = impl->CreateCroppedPixelBuffer(pixel_buffer, out_width,
                                                      out_height, pixel_format);
      if (cropped_buffer) {
        present_buffer = cropped_buffer;
      } else {
        out_width = width;
        out_height = height;
      }
    }
    // All Apple consumers can present the IOSurface-backed image directly.
    // The descriptor remains borrowed and is valid only during this callback.
    native_frame.struct_size = sizeof(native_frame);
    native_frame.type = MiniRtcNativeVideoFrameCVPixelBuffer;
    native_frame.width = out_width;
    native_frame.height = out_height;
    native_frame.payload.cv_pixel_buffer = present_buffer;
    native_frame.owner = present_buffer;
    native_frame.retain = &RetainPixelBufferOwner;
    native_frame.release = &ReleasePixelBufferOwner;
    native_frame.copy_to_nv12 = &CopyPixelBufferToNv12;
    decoded_frame.SetNativeFrame(&native_frame);
    decoded_frame.SetSize(static_cast<size_t>(out_width) * out_height * 3U / 2U);
  } else {
    CVReturn lock_status =
        CVPixelBufferLockBaseAddress(pixel_buffer, kCVPixelBufferLock_ReadOnly);
    if (lock_status != kCVReturnSuccess) {
      LOG_ERROR("Failed to lock decoded pixel buffer: {}", lock_status);
      return;
    }

    const size_t y_stride = CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer, 0);
    const size_t uv_stride = CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer, 1);
    const uint8_t* y_plane = static_cast<const uint8_t*>(
        CVPixelBufferGetBaseAddressOfPlane(pixel_buffer, 0));
    const uint8_t* uv_plane = static_cast<const uint8_t*>(
        CVPixelBufferGetBaseAddressOfPlane(pixel_buffer, 1));

    if (!y_plane || !uv_plane || y_stride < out_width || uv_stride < out_width) {
      LOG_ERROR("Invalid decoded NV12 planes: {}x{}, strides {} / {}", out_width,
                out_height, y_stride, uv_stride);
      CVPixelBufferUnlockBaseAddress(pixel_buffer, kCVPixelBufferLock_ReadOnly);
      return;
    }

    const size_t packed_size = static_cast<size_t>(out_width) * out_height * 3 / 2;
    std::vector<uint8_t> packed_nv12(packed_size);
    for (size_t row = 0; row < out_height; ++row) {
      memcpy(packed_nv12.data() + row * out_width, y_plane + row * y_stride,
             out_width);
    }
    uint8_t* uv_dst =
        packed_nv12.data() + static_cast<size_t>(out_width) * out_height;
    for (size_t row = 0; row < out_height / 2; ++row) {
      memcpy(uv_dst + row * out_width, uv_plane + row * uv_stride, out_width);
    }
    CVPixelBufferUnlockBaseAddress(pixel_buffer,
                                   kCVPixelBufferLock_ReadOnly);

    decoded_frame.UpdateBuffer(packed_nv12.data(), packed_nv12.size());
  }
  decoded_frame.SetDecodedWidth(out_width);
  decoded_frame.SetDecodedHeight(out_height);
  decoded_frame.SetCapturedTimestamp(timestamps.captured_us);
  decoded_frame.SetReceivedTimestamp(timestamps.received_us);
  decoded_frame.SetDecodedTimestamp(impl->clock_->CurrentTimeUs());

  const uint64_t frame_count = ++impl->decoded_frame_count_;
  if (frame_count == 1 || frame_count % 300 == 0) {
    LOG_DEBUG("VideoToolbox decoded frame {}, size={}x{}, pixel_format={}",
             frame_count, out_width, out_height, pixel_format);
  }

  if (impl->on_receive_decoded_frame_) {
#ifdef SAVE_DECODED_NV12_STREAM
    fwrite((unsigned char*)decoded_frame.Buffer(), 1, decoded_frame.Size(),
           impl->file_nv12_);
#endif
    impl->on_receive_decoded_frame_(&decoded_frame);
  }
  if (cropped_buffer) {
    CFRelease(cropped_buffer);
  }
}

//

VideoToolboxDecoder::VideoToolboxDecoder(std::shared_ptr<SystemClock> clock,
                                         bool native_video_output,
                                         VideoCodecType codec_type)
    : impl_(std::make_shared<Impl>(std::move(clock), native_video_output,
                                   codec_type)) {}

VideoToolboxDecoder::~VideoToolboxDecoder() = default;

int VideoToolboxDecoder::Init() { return impl_->Init(); }

int VideoToolboxDecoder::Decode(std::unique_ptr<ReceivedFrame> received_frame,
                                std::function<void(const DecodedFrame*)> on_receive_decoded_frame) {
  return impl_->Decode(std::move(received_frame), std::move(on_receive_decoded_frame));
}

std::string VideoToolboxDecoder::GetDecoderName() const { return impl_->GetDecoderName(); }

}  // namespace minirtc
