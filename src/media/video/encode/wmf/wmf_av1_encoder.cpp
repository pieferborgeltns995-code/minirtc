#include "wmf_av1_encoder.h"

#include <codecapi.h>
#include <strmif.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <limits>
#include <thread>
#include <vector>

#include "avt/svt_av1_encoder.h"
#include "log.h"
#include "nv12_scaler.h"
#include "video/av1_crop_metadata.h"
#include "wmf/wmf_utils.h"

namespace minirtc {
namespace {
using wmf::ComPtr;

// MFVideoFormat_AV1 ('AV01') defined explicitly for broad Windows SDK compat.
constexpr GUID kVideoFormatAv1 = {
    0x31305641,
    0x0000,
    0x0010,
    {0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71}};

constexpr uint8_t kObuTypeSequenceHeader = 1;
constexpr uint8_t kObuTypeTemporalDelimiter = 2;
constexpr uint8_t kObuTypeFrameHeader = 3;
constexpr uint8_t kObuTypeFrame = 6;

HRESULT SetUint(ICodecAPI* api, const GUID& key, ULONG value) {
  VARIANT variant{};
  variant.vt = VT_UI4;
  variant.ulVal = value;
  return api->SetValue(&key, &variant);
}

HRESULT EnumerateEncoders(IMFActivate*** activates, UINT32* count) {
  MFT_REGISTER_TYPE_INFO input{MFMediaType_Video, MFVideoFormat_NV12};
  MFT_REGISTER_TYPE_INFO output{MFMediaType_Video, kVideoFormatAv1};
  HRESULT hr = MFTEnumEx(
      MFT_CATEGORY_VIDEO_ENCODER,
      MFT_ENUM_FLAG_HARDWARE | MFT_ENUM_FLAG_SORTANDFILTER, &input, &output,
      activates, count);
  if (SUCCEEDED(hr) && *count > 0) {
    return hr;
  }
  if (SUCCEEDED(hr) && *activates) {
    CoTaskMemFree(*activates);
    *activates = nullptr;
    *count = 0;
  }
  // Some drivers register AV1 MFTs without advertising NV12 in the static
  // registry input list even though NV12 is accepted at SetInputType time.
  return MFTEnumEx(MFT_CATEGORY_VIDEO_ENCODER,
                   MFT_ENUM_FLAG_HARDWARE | MFT_ENUM_FLAG_SORTANDFILTER,
                   nullptr, &output, activates, count);
}

bool ValidConfig(const MediaCodecConfig& config) {
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

int AlignDimDown(int value, int align, int min_value) {
  if (value <= min_value) return min_value;
  return (value / align) * align;
}

int AlignDimUp(int value, int align) {
  if (value <= 0) return align;
  return ((value + align - 1) / align) * align;
}

std::vector<EncodeSize> BuildCandidateEncodeSizes(int src_w, int src_h) {
  std::vector<EncodeSize> candidates;
  auto push_unique = [&](int w, int h, EncodeMode mode) {
    if (w <= 0 || h <= 0 || w > 8192 || h > 8192 ||
        (w & 1) != 0 || (h & 1) != 0) {
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
    const double scale = std::min(3840.0 / static_cast<double>(src_w),
                                  2160.0 / static_cast<double>(src_h));
    push_unique(AlignDimDown(static_cast<int>(src_w * scale), 64, 64),
                AlignDimDown(static_cast<int>(src_h * scale), 16, 16),
                EncodeMode::kScale);
  }
  return candidates;
}

bool PadNv12EdgeClamp(const uint8_t* src, int src_w, int src_h, uint8_t* dst,
                      int dst_w, int dst_h) {
  if (!src || !dst || src_w <= 0 || src_h <= 0 || dst_w < src_w ||
      dst_h < src_h || (src_w & 1) != 0 || (src_h & 1) != 0 ||
      (dst_w & 1) != 0 || (dst_h & 1) != 0) {
    return false;
  }
  const int pad_right = dst_w - src_w;
  for (int y = 0; y < src_h; ++y) {
    const uint8_t* src_row = src + static_cast<size_t>(y) * src_w;
    uint8_t* dst_row = dst + static_cast<size_t>(y) * dst_w;
    std::memcpy(dst_row, src_row, static_cast<size_t>(src_w));
    if (pad_right > 0) {
      std::memset(dst_row + src_w, src_row[src_w - 1],
                  static_cast<size_t>(pad_right));
    }
  }
  const uint8_t* last_y = dst + static_cast<size_t>(src_h - 1) * dst_w;
  for (int y = src_h; y < dst_h; ++y) {
    std::memcpy(dst + static_cast<size_t>(y) * dst_w, last_y,
                static_cast<size_t>(dst_w));
  }

  const uint8_t* src_uv = src + static_cast<size_t>(src_w) * src_h;
  uint8_t* dst_uv = dst + static_cast<size_t>(dst_w) * dst_h;
  const int src_uv_h = src_h / 2;
  const int dst_uv_h = dst_h / 2;
  for (int y = 0; y < src_uv_h; ++y) {
    const uint8_t* src_row = src_uv + static_cast<size_t>(y) * src_w;
    uint8_t* dst_row = dst_uv + static_cast<size_t>(y) * dst_w;
    std::memcpy(dst_row, src_row, static_cast<size_t>(src_w));
    const uint8_t u = src_row[src_w - 2];
    const uint8_t v = src_row[src_w - 1];
    for (int x = src_w; x < dst_w; x += 2) {
      dst_row[x] = u;
      dst_row[x + 1] = v;
    }
  }
  const uint8_t* last_uv = dst_uv + static_cast<size_t>(src_uv_h - 1) * dst_w;
  for (int y = src_uv_h; y < dst_uv_h; ++y) {
    std::memcpy(dst_uv + static_cast<size_t>(y) * dst_w, last_uv,
                static_cast<size_t>(dst_w));
  }
  return true;
}

struct Av1ObuSpan {
  size_t offset = 0;
  size_t size = 0;
  size_t payload_offset = 0;
  size_t payload_size = 0;
  uint8_t type = 0;
};

bool ReadLeb128(const uint8_t* data, size_t size, size_t& pos,
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

std::vector<Av1ObuSpan> ParseAv1Obus(const uint8_t* data, size_t size) {
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

}  // namespace

class WmfAv1Encoder::Impl {
 public:
  explicit Impl(std::shared_ptr<SystemClock> clock)
      : clock_(std::move(clock)) {}
  ~Impl() {
    worker_.Call([this] {
      Reset();
      fallback_.reset();
    });
  }

  void Reset() {
    events_.Reset();
    api_.Reset();
    wmf::ShutdownTransform(encoder_);
    input_credit_ = 0;
    sequence_header_.clear();
    sequence_ = 0;
    force_idr_ = true;
  }

  HRESULT Configure(IMFActivate* activation, int width, int height) {
    HRESULT hr = activation->ActivateObject(IID_PPV_ARGS(&encoder_));
    if (FAILED(hr)) return hr;
    ComPtr<IMFAttributes> attributes;
    hr = encoder_->GetAttributes(&attributes);
    if (FAILED(hr)) return hr;
    async_ =
        MFGetAttributeUINT32(attributes.Get(), MF_TRANSFORM_ASYNC, FALSE) != 0;
    if (async_) {
      hr = attributes->SetUINT32(MF_TRANSFORM_ASYNC_UNLOCK, TRUE);
      if (FAILED(hr)) return hr;
      hr = encoder_.As(&events_);
      if (FAILED(hr)) return hr;
    }
    DWORD inputs = 0, outputs = 0;
    hr = encoder_->GetStreamCount(&inputs, &outputs);
    if (FAILED(hr) || inputs != 1 || outputs != 1) return E_UNEXPECTED;
    input_id_ = output_id_ = 0;
    hr = encoder_->GetStreamIDs(1, &input_id_, 1, &output_id_);
    if (FAILED(hr) && hr != E_NOTIMPL) return hr;

    if (SUCCEEDED(encoder_.As(&api_)) && api_) {
      HRESULT rc_hr = SetUint(api_.Get(), CODECAPI_AVEncCommonRateControlMode,
                              eAVEncCommonRateControlMode_CBR);
      if (FAILED(rc_hr)) {
        SetUint(api_.Get(), CODECAPI_AVEncCommonRateControlMode,
                eAVEncCommonRateControlMode_UnconstrainedVBR);
      }
      VARIANT low_latency{};
      low_latency.vt = VT_BOOL;
      low_latency.boolVal = VARIANT_TRUE;
      api_->SetValue(&CODECAPI_AVLowLatencyMode, &low_latency);
      SetUint(api_.Get(), CODECAPI_AVEncMPVDefaultBPictureCount, 0);
      SetUint(api_.Get(), CODECAPI_AVEncMPVGOPSize, config_.key_frame_interval);
      SetUint(api_.Get(), CODECAPI_AVEncCommonQualityVsSpeed,
              config_.video_degradation_preference ==
                      MiniRtcVideoDegradationPreference::MaintainFrameRate
                  ? 0
                  : 50);
    }

    ComPtr<IMFMediaType> output, input;
    hr = MFCreateMediaType(&output);
    if (FAILED(hr)) return hr;
    output->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    output->SetGUID(MF_MT_SUBTYPE, kVideoFormatAv1);
    output->SetUINT32(MF_MT_AVG_BITRATE, config_.init_bitrate);
    output->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
    MFSetAttributeSize(output.Get(), MF_MT_FRAME_SIZE, width, height);
    MFSetAttributeRatio(output.Get(), MF_MT_FRAME_RATE, config_.max_frame_rate,
                        1);
    MFSetAttributeRatio(output.Get(), MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    hr = encoder_->SetOutputType(output_id_, output.Get(), 0);
    if (FAILED(hr)) {
      LOG_WARN("WMF AV1 SetOutputType failed (0x{:08x}); retrying with profile=0",
               static_cast<uint32_t>(hr));
      output->SetUINT32(MF_MT_MPEG2_PROFILE, 0);
      hr = encoder_->SetOutputType(output_id_, output.Get(), 0);
      if (FAILED(hr)) return hr;
    }

    hr = MFCreateMediaType(&input);
    if (FAILED(hr)) return hr;
    output->CopyAllItems(input.Get());
    input->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_NV12);
    input->DeleteItem(MF_MT_MPEG2_PROFILE);
    input->DeleteItem(MF_MT_AVG_BITRATE);
    input->SetUINT32(MF_MT_DEFAULT_STRIDE, width);
    hr = encoder_->SetInputType(input_id_, input.Get(), 0);
    if (FAILED(hr)) {
      LOG_WARN("WMF AV1 SetInputType failed (0x{:08x}); retrying without stride",
               static_cast<uint32_t>(hr));
      input->DeleteItem(MF_MT_DEFAULT_STRIDE);
      hr = encoder_->SetInputType(input_id_, input.Get(), 0);
      if (FAILED(hr)) return hr;
    }

    if (api_) {
      HRESULT br_hr = SetUint(api_.Get(), CODECAPI_AVEncCommonMeanBitRate,
                              config_.init_bitrate);
      if (FAILED(br_hr)) {
        LOG_DEBUG("WMF AV1 SetUint MeanBitRate failed: 0x{:08x}",
                  static_cast<uint32_t>(br_hr));
      }
    }
    hr = encoder_->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, 0);
    if (FAILED(hr)) return hr;
    return encoder_->ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM, 0);
  }

  HRESULT Initialize() {
    Reset();
    if (FAILED(worker_.Status())) return worker_.Status();
    IMFActivate** activations = nullptr;
    UINT32 count = 0;
    HRESULT hr = EnumerateEncoders(&activations, &count);
    if (FAILED(hr)) return hr;
    const auto candidates = BuildCandidateEncodeSizes(source_width_, source_height_);
    hr = MF_E_TOPO_CODEC_NOT_FOUND;
    for (UINT32 i = 0; i < count; ++i) {
      if (!encoder_) {
        HRESULT last_hr = MF_E_TOPO_CODEC_NOT_FOUND;
        for (const auto& candidate : candidates) {
          last_hr = Configure(activations[i], candidate.width, candidate.height);
          if (SUCCEEDED(last_hr)) {
            encode_width_ = candidate.width;
            encode_height_ = candidate.height;
            encode_mode_ = candidate.mode;
            break;
          }
          LOG_DEBUG("Hardware AV1 MFT config {}x{} failed: 0x{:08x}",
                    candidate.width, candidate.height,
                    static_cast<uint32_t>(last_hr));
          Reset();
          activations[i]->ShutdownObject();
        }
        hr = last_hr;
        if (FAILED(hr)) {
          LOG_WARN("Hardware AV1 MFT initialization failed: 0x{:08x}",
                   static_cast<uint32_t>(hr));
        } else {
          WCHAR name[256]{};
          activations[i]->GetString(MFT_FRIENDLY_NAME_Attribute, name, 256,
                                    nullptr);
          char utf8[1024]{};
          WideCharToMultiByte(CP_UTF8, 0, name, -1, utf8, sizeof(utf8), nullptr,
                              nullptr);
          LOG_INFO("Media Foundation hardware AV1 encoder: {}", utf8);
          if (encode_mode_ == EncodeMode::kPad) {
            LOG_INFO(
                "Media Foundation hardware AV1 encoder padding {}x{} -> {}x{} "
                "(1:1 crop metadata)",
                source_width_, source_height_, encode_width_, encode_height_);
          } else if (encode_mode_ == EncodeMode::kScale) {
            LOG_INFO(
                "Media Foundation hardware AV1 encoder scaling {}x{} -> {}x{}",
                source_width_, source_height_, encode_width_, encode_height_);
          }
          activations[i]->DetachObject();
        }
      }
      activations[i]->Release();
    }
    CoTaskMemFree(activations);
    reconfigure_ = false;
    return encoder_ ? S_OK : hr;
  }

  void CacheSequenceHeaderFromBytes(const uint8_t* data, size_t size) {
    for (const auto& obu : ParseAv1Obus(data, size)) {
      if (obu.type == kObuTypeSequenceHeader) {
        sequence_header_.assign(data + obu.offset,
                                data + obu.offset + obu.size);
        return;
      }
    }
  }

  HRESULT ReadFrame(IMFSample* sample, const RawFrame& raw,
                    std::vector<EncodedFrame>& frames) {
    std::vector<uint8_t> bytes;
    HRESULT hr = wmf::ReadBytes(sample, bytes);
    if (FAILED(hr)) return hr;
    if (bytes.empty()) return MF_E_INVALID_STREAM_DATA;

    const auto obus = ParseAv1Obus(bytes.data(), bytes.size());
    if (obus.empty()) {
      LOG_WARN("WMF AV1 output is not a valid OBU stream (size={})",
               bytes.size());
      return MF_E_INVALID_STREAM_DATA;
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
        sequence_header_.assign(bytes.data() + obu.offset,
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

    UINT32 clean_point = 0;
    const bool sample_clean =
        SUCCEEDED(sample->GetUINT32(MFSampleExtension_CleanPoint,
                                    &clean_point)) &&
        clean_point != 0;
    const bool key = sample_clean || has_seq_header || obu_is_key;

    if (key && !has_seq_header) {
      if (sequence_header_.empty()) {
        ComPtr<IMFMediaType> type;
        UINT32 size = 0;
        if (SUCCEEDED(encoder_->GetOutputCurrentType(output_id_, &type)) &&
            SUCCEEDED(type->GetBlobSize(MF_MT_MPEG_SEQUENCE_HEADER, &size)) &&
            size > 0) {
          std::vector<uint8_t> blob(size);
          if (SUCCEEDED(type->GetBlob(MF_MT_MPEG_SEQUENCE_HEADER, blob.data(),
                                      size, nullptr))) {
            CacheSequenceHeaderFromBytes(blob.data(), blob.size());
          }
        }
      }
      if (!sequence_header_.empty()) {
        bytes.insert(bytes.begin() + static_cast<ptrdiff_t>(insert_offset),
                     sequence_header_.begin(), sequence_header_.end());
        after_seq_offset = insert_offset + sequence_header_.size();
      }
    }
    if (key && encode_mode_ == EncodeMode::kPad && source_width_ > 0 &&
        source_height_ > 0 && source_width_ <= 65535 &&
        source_height_ <= 65535) {
      av1_crop::AppendCropMetadataObu(bytes, after_seq_offset,
                                      static_cast<uint16_t>(source_width_),
                                      static_cast<uint16_t>(source_height_));
    }

    const uint32_t out_w = static_cast<uint32_t>(VisibleWidth());
    const uint32_t out_h = static_cast<uint32_t>(VisibleHeight());
    EncodedFrame frame(bytes.data(), bytes.size(), out_w, out_h);
    frame.SetEncodedWidth(out_w);
    frame.SetEncodedHeight(out_h);
    frame.SetFrameType(key ? kVideoFrameKey : kVideoFrameDelta);
    frame.SetCapturedTimestamp(raw.CapturedTimestamp());
    frame.SetEncodedTimestamp(clock_->CurrentTime());
    frames.push_back(std::move(frame));
    if (key) {
      force_idr_ = false;
    }
    return S_OK;
  }

  int VisibleWidth() const {
    return encode_mode_ == EncodeMode::kScale ? encode_width_ : source_width_;
  }
  int VisibleHeight() const {
    return encode_mode_ == EncodeMode::kScale ? encode_height_ : source_height_;
  }

  HRESULT EncodeHardware(const RawFrame& raw,
                         std::vector<EncodedFrame>& frames) {
    if (static_cast<int>(raw.Width()) != source_width_ ||
        static_cast<int>(raw.Height()) != source_height_ || reconfigure_) {
      source_width_ = static_cast<int>(raw.Width());
      source_height_ = static_cast<int>(raw.Height());
      config_.init_width = source_width_;
      config_.init_height = source_height_;
      HRESULT hr = Initialize();
      if (FAILED(hr)) return hr;
    }
    if (!encoder_) return MF_E_NOT_INITIALIZED;
    if (sequence_ % config_.key_frame_interval == 0) force_idr_ = true;
    if (force_idr_ && api_) {
      HRESULT kf_hr = SetUint(api_.Get(), CODECAPI_AVEncVideoForceKeyFrame, 1);
      if (FAILED(kf_hr)) {
        LOG_DEBUG("WMF AV1 ForceKeyFrame returned 0x{:08x}",
                  static_cast<uint32_t>(kf_hr));
      }
    }
    ComPtr<IMFSample> input;
    const LONGLONG duration = 10'000'000LL / config_.max_frame_rate;
    const uint8_t* sample_buffer = raw.Buffer();
    size_t input_size = 0;
    if (encode_mode_ == EncodeMode::kPad) {
      if (!GetNv12FrameSize(encode_width_, encode_height_, &input_size)) {
        return E_INVALIDARG;
      }
      scaled_nv12_.resize(input_size);
      if (!PadNv12EdgeClamp(raw.Buffer(), source_width_, source_height_,
                            scaled_nv12_.data(), encode_width_,
                            encode_height_)) {
        return E_FAIL;
      }
      sample_buffer = scaled_nv12_.data();
    } else if (encode_mode_ == EncodeMode::kScale) {
      if (!GetNv12FrameSize(encode_width_, encode_height_, &input_size)) {
        return E_INVALIDARG;
      }
      scaled_nv12_.resize(input_size);
      const uint8_t* src_y = raw.Buffer();
      const uint8_t* src_uv =
          src_y + static_cast<size_t>(source_width_) * source_height_;
      uint8_t* dst_y = scaled_nv12_.data();
      uint8_t* dst_uv =
          dst_y + static_cast<size_t>(encode_width_) * encode_height_;
      if (ScaleNv12ViaI420(src_y, source_width_, src_uv, source_width_,
                           source_width_, source_height_, dst_y, encode_width_,
                           dst_uv, encode_width_, encode_width_, encode_height_,
                           libyuv::kFilterBilinear, &scaler_scratch_) != 0) {
        return E_FAIL;
      }
      sample_buffer = scaled_nv12_.data();
    } else {
      GetNv12FrameSize(raw.Width(), raw.Height(), &input_size);
    }
    HRESULT hr = wmf::CreateSample(sample_buffer, input_size, next_time_,
                                   duration, input);
    if (FAILED(hr)) return hr;
    next_time_ += duration;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    bool submitted = false;
    while (std::chrono::steady_clock::now() < deadline) {
      bool have_output = !async_;
      bool received_event = false;
      if (async_) {
        ComPtr<IMFMediaEvent> event;
        hr = events_->GetEvent(MF_EVENT_FLAG_NO_WAIT, &event);
        if (hr != MF_E_NO_EVENTS_AVAILABLE) {
          if (FAILED(hr)) return hr;
          received_event = true;
          HRESULT status = S_OK;
          hr = event->GetStatus(&status);
          if (FAILED(hr)) return hr;
          if (FAILED(status)) return status;
          MediaEventType type = MEUnknown;
          hr = event->GetType(&type);
          if (FAILED(hr)) return hr;
          if (type == METransformNeedInput) ++input_credit_;
          if (type == METransformHaveOutput) have_output = true;
        }
      }
      if (!submitted && (!async_ || input_credit_)) {
        hr = encoder_->ProcessInput(input_id_, input.Get(), 0);
        if (SUCCEEDED(hr)) {
          submitted = true;
          if (async_) --input_credit_;
        } else if (hr != MF_E_NOTACCEPTING) {
          return hr;
        }
      }
      if (have_output) {
        ComPtr<IMFSample> output;
        hr = wmf::ReadOutput(encoder_.Get(), output_id_, output);
        if (hr == MF_E_TRANSFORM_STREAM_CHANGE) {
          ComPtr<IMFMediaType> type;
          hr = encoder_->GetOutputAvailableType(output_id_, 0, &type);
          if (FAILED(hr)) return hr;
          hr = encoder_->SetOutputType(output_id_, type.Get(), 0);
          if (FAILED(hr)) return hr;
          if (async_) continue;
          hr = wmf::ReadOutput(encoder_.Get(), output_id_, output);
        }
        if (FAILED(hr) && hr != MF_E_TRANSFORM_NEED_MORE_INPUT) return hr;
        if (SUCCEEDED(hr) && output) {
          hr = ReadFrame(output.Get(), raw, frames);
          if (FAILED(hr)) return hr;
          ++sequence_;
          return S_OK;
        }
      }
      if (!received_event) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    return HRESULT_FROM_WIN32(WAIT_TIMEOUT);
  }

  wmf::CodecThread worker_;
  std::shared_ptr<SystemClock> clock_;
  MediaCodecConfig config_;
  ComPtr<IMFTransform> encoder_;
  ComPtr<IMFMediaEventGenerator> events_;
  ComPtr<ICodecAPI> api_;
  DWORD input_id_ = 0, output_id_ = 0;
  unsigned input_credit_ = 0;
  bool async_ = false, force_idr_ = true, reconfigure_ = false;
  bool initialized_ = false;
  int source_width_ = 0, source_height_ = 0;
  int encode_width_ = 0, encode_height_ = 0;
  EncodeMode encode_mode_ = EncodeMode::kNative;
  uint64_t sequence_ = 0;
  LONGLONG next_time_ = 0;
  std::vector<uint8_t> sequence_header_;
  std::vector<uint8_t> scaled_nv12_;
  std::vector<uint8_t> scaler_scratch_;
  std::unique_ptr<SvtAv1Encoder> fallback_;
};

WmfAv1Encoder::WmfAv1Encoder(std::shared_ptr<SystemClock> clock)
    : impl_(std::make_unique<Impl>(std::move(clock))) {}
WmfAv1Encoder::~WmfAv1Encoder() = default;

bool WmfAv1Encoder::IsSupported() {
  wmf::CodecThread worker;
  if (FAILED(worker.Status())) return false;
  return worker.Call([] {
    IMFActivate** activations = nullptr;
    UINT32 count = 0;
    const HRESULT hr = EnumerateEncoders(&activations, &count);
    for (UINT32 i = 0; i < count; ++i) activations[i]->Release();
    CoTaskMemFree(activations);
    return SUCCEEDED(hr) && count > 0;
  });
}

int WmfAv1Encoder::Init(const MediaCodecConfig& config) {
  if (!ValidConfig(config)) return -1;
  return impl_->worker_.Call([&] {
    impl_->fallback_.reset();
    impl_->config_ = config;
    impl_->config_.init_bitrate =
        ClampEncoderTargetBitrate(config.init_bitrate, config.max_bitrate);
    if (impl_->config_.average_bitrate <= 0) {
      impl_->config_.average_bitrate = impl_->config_.init_bitrate;
    }
    impl_->source_width_ = impl_->config_.init_width;
    impl_->source_height_ = impl_->config_.init_height;
    impl_->initialized_ = SUCCEEDED(impl_->Initialize());
    return impl_->initialized_ ? 0 : -1;
  });
}

int WmfAv1Encoder::Encode(const RawFrame& raw,
                          std::function<int(const EncodedFrame&)> callback) {
  size_t size = 0;
  if (!raw.Buffer() || raw.Width() > 8192 || raw.Height() > 8192 ||
      !GetNv12FrameSize(raw.Width(), raw.Height(), &size) || raw.Size() < size)
    return -1;
  std::vector<EncodedFrame> frames;
  const int result = impl_->worker_.Call([&] {
    if (!impl_->initialized_) return -1;
    if (!impl_->fallback_) {
      const HRESULT hr = impl_->EncodeHardware(raw, frames);
      if (SUCCEEDED(hr)) return 0;
      LOG_WARN(
          "Media Foundation hardware AV1 encoding failed (0x{:08x}); falling "
          "back to SVT-AV1",
          static_cast<uint32_t>(hr));
      impl_->Reset();
      impl_->fallback_ = std::make_unique<SvtAv1Encoder>(impl_->clock_);
      if (impl_->fallback_->Init(impl_->config_) != 0) {
        impl_->fallback_.reset();
        return -1;
      }
    }
    return impl_->fallback_->Encode(raw, [&](const EncodedFrame& frame) {
      frames.push_back(frame);
      return 0;
    });
  });
  for (const auto& frame : frames)
    if (callback) callback(frame);
  return result;
}

int WmfAv1Encoder::ForceIdr() {
  return impl_->worker_.Call([&] {
    impl_->force_idr_ = true;
    return impl_->fallback_ ? impl_->fallback_->ForceIdr() : 0;
  });
}

int WmfAv1Encoder::SetTargetBitrate(int bitrate) {
  if (bitrate <= 0) return -1;
  return impl_->worker_.Call([&] {
    bitrate = ClampEncoderTargetBitrate(bitrate, impl_->config_.max_bitrate);
    if (impl_->fallback_) {
      const int result = impl_->fallback_->SetTargetBitrate(bitrate);
      if (result == 0) impl_->config_.init_bitrate = bitrate;
      return result;
    }
    if (!impl_->api_) return -1;
    if (FAILED(SetUint(impl_->api_.Get(), CODECAPI_AVEncCommonMeanBitRate,
                       bitrate)))
      impl_->reconfigure_ = true;
    impl_->config_.init_bitrate = bitrate;
    return 0;
  });
}

int WmfAv1Encoder::SetTargetFrameRate(int frame_rate) {
  if (frame_rate <= 0 || frame_rate > 240) return -1;
  return impl_->worker_.Call([&] {
    if (impl_->fallback_) {
      const int result = impl_->fallback_->SetTargetFrameRate(frame_rate);
      if (result == 0) impl_->config_.max_frame_rate = frame_rate;
      return result;
    }
    if (!impl_->encoder_) return -1;
    if (impl_->config_.max_frame_rate != frame_rate) impl_->reconfigure_ = true;
    impl_->config_.max_frame_rate = frame_rate;
    return 0;
  });
}

int WmfAv1Encoder::GetResolution(int* width, int* height) const {
  if (!width || !height) return -1;
  return impl_->worker_.Call([&] {
    if (impl_->fallback_) return impl_->fallback_->GetResolution(width, height);
    if (!impl_->encoder_) return -1;
    *width = impl_->VisibleWidth();
    *height = impl_->VisibleHeight();
    return 0;
  });
}

std::string WmfAv1Encoder::GetEncoderName() const {
  return impl_->worker_.Call([&]() -> std::string {
    return impl_->fallback_ ? impl_->fallback_->GetEncoderName()
                            : "Media Foundation (AV1, HW)";
  });
}

}  // namespace minirtc
