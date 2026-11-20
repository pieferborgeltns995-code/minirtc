#include "wmf_av1_encoder.h"

#include <codecapi.h>
#include <d3d11.h>
#include <mfapi.h>
#include <strmif.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <thread>
#include <vector>

#include "av1_encode_helpers.h"
#include "avt/svt_av1_encoder.h"
#include "log.h"
#include "wmf/wmf_utils.h"

namespace minirtc {
namespace {
using av1_encode::BuildCandidateEncodeSizes;
using av1_encode::CacheSequenceHeaderFromBytes;
using av1_encode::EncodeMode;
using av1_encode::EncodeStatsTracker;
using av1_encode::PrepareNv12ToStridedPlanes;
using av1_encode::ProcessEncodedAv1Bitstream;
using av1_encode::PushEncodedAv1Frame;
using av1_encode::ResolveCpuNv12;
using av1_encode::ValidConfig;
using wmf::ComPtr;

// MFVideoFormat_AV1 ('AV01') defined explicitly for broad Windows SDK compat.
constexpr GUID kVideoFormatAv1 = {
    0x31305641,
    0x0000,
    0x0010,
    {0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71}};

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

}  // namespace

class WmfAv1Encoder::Impl {
 public:
  explicit Impl(std::shared_ptr<SystemClock> clock)
      : clock_(std::move(clock)) {}
  ~Impl() {
    callback_worker_.Flush();
    worker_.Call([this] {
      Reset();
      d3d_device_.Reset();
      d3d_tex_width_ = 0;
      d3d_tex_height_ = 0;
      fallback_.reset();
    });
  }

  void Reset() {
    if (encoder_ && using_d3d11_) {
      encoder_->ProcessMessage(MFT_MESSAGE_SET_D3D_MANAGER, 0);
    }
    dxgi_buffer_cache_.clear();
    reusable_input_sample_.Reset();
    reusable_output_sample_.Reset();
    dxgi_manager_.Reset();
    dxgi_reset_token_ = 0;
    using_d3d11_ = false;
    reusable_cpu_buffer_.Reset();
    reusable_cpu_buffer_size_ = 0;
    events_.Reset();
    api_.Reset();
    wmf::ShutdownTransform(encoder_);
    input_credit_ = 0;
    sequence_header_.clear();
    sequence_ = 0;
    force_idr_ = true;
  }

  HRESULT GetOrCreateDxgiBuffer(ID3D11Texture2D* tex, uint32_t subresource,
                                ComPtr<IMFMediaBuffer>& out_buffer) {
    if (!tex) return E_INVALIDARG;
    for (auto& entry : dxgi_buffer_cache_) {
      if (entry.texture == tex && entry.subresource == subresource) {
        out_buffer = entry.buffer;
        return S_OK;
      }
    }
    ComPtr<IMFMediaBuffer> dxgi_buffer;
    HRESULT hr = MFCreateDXGISurfaceBuffer(
        __uuidof(ID3D11Texture2D), tex, subresource, FALSE, &dxgi_buffer);
    if (FAILED(hr)) return hr;
    DWORD max_len = 0;
    if (SUCCEEDED(dxgi_buffer->GetMaxLength(&max_len)) && max_len > 0) {
      dxgi_buffer->SetCurrentLength(max_len);
    }
    if (dxgi_buffer_cache_.size() >= 8) {
      dxgi_buffer_cache_.erase(dxgi_buffer_cache_.begin());
    }
    dxgi_buffer_cache_.push_back({tex, subresource, dxgi_buffer});
    out_buffer = std::move(dxgi_buffer);
    return S_OK;
  }

  HRESULT PrepareInputSample(IMFMediaBuffer* buffer, LONGLONG sample_time,
                             LONGLONG duration, ComPtr<IMFSample>& out_sample) {
    if (!buffer) return E_INVALIDARG;
    HRESULT hr = S_OK;
    if (!reusable_input_sample_) {
      hr = MFCreateSample(&reusable_input_sample_);
      if (FAILED(hr)) return hr;
    } else {
      hr = reusable_input_sample_->RemoveAllBuffers();
      if (FAILED(hr)) return hr;
    }
    hr = reusable_input_sample_->AddBuffer(buffer);
    if (FAILED(hr)) return hr;
    hr = reusable_input_sample_->SetSampleTime(sample_time);
    if (FAILED(hr)) return hr;
    hr = reusable_input_sample_->SetSampleDuration(duration);
    if (FAILED(hr)) return hr;
    out_sample = reusable_input_sample_;
    return S_OK;
  }

  HRESULT Configure(IMFActivate* activation, int width, int height,
                    ID3D11Device* d3d_device) {
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
    if (d3d_device) {
      if (MFGetAttributeUINT32(attributes.Get(), MF_SA_D3D11_AWARE, FALSE) ==
          0) {
        return MF_E_UNSUPPORTED_D3D_TYPE;
      }
      hr = MFCreateDXGIDeviceManager(&dxgi_reset_token_, &dxgi_manager_);
      if (FAILED(hr)) return hr;
      hr = dxgi_manager_->ResetDevice(d3d_device, dxgi_reset_token_);
      if (FAILED(hr)) return hr;
      hr = encoder_->ProcessMessage(
          MFT_MESSAGE_SET_D3D_MANAGER,
          reinterpret_cast<ULONG_PTR>(dxgi_manager_.Get()));
      if (FAILED(hr)) return hr;
      using_d3d11_ = true;
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
      SetUint(api_.Get(), CODECAPI_AVEncCommonQualityVsSpeed, 0);
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
    if (!using_d3d11_) {
      input->SetUINT32(MF_MT_DEFAULT_STRIDE, width);
    }
    hr = encoder_->SetInputType(input_id_, input.Get(), 0);
    if (FAILED(hr) && !using_d3d11_) {
      LOG_WARN("WMF AV1 SetInputType failed (0x{:08x}); retrying without stride",
               static_cast<uint32_t>(hr));
      input->DeleteItem(MF_MT_DEFAULT_STRIDE);
      hr = encoder_->SetInputType(input_id_, input.Get(), 0);
    }
    if (FAILED(hr)) return hr;

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
        if (d3d_device_ && d3d_tex_width_ > 0 && d3d_tex_height_ > 0) {
          last_hr = Configure(activations[i], d3d_tex_width_, d3d_tex_height_,
                              d3d_device_.Get());
          if (SUCCEEDED(last_hr)) {
            d3d11_failed_device_ = nullptr;
            encode_width_ = d3d_tex_width_;
            encode_height_ = d3d_tex_height_;
            encode_mode_ = (d3d_tex_width_ != source_width_ ||
                            d3d_tex_height_ != source_height_)
                               ? EncodeMode::kPad
                               : EncodeMode::kNative;
          } else {
            LOG_WARN(
                "Hardware AV1 MFT D3D11 config {}x{} failed (0x{:08x}); "
                "falling back to CPU buffer input",
                d3d_tex_width_, d3d_tex_height_,
                static_cast<uint32_t>(last_hr));
            d3d11_failed_device_ = d3d_device_.Get();
            Reset();
            activations[i]->ShutdownObject();
          }
        }
        if (!encoder_) {
          d3d_device_.Reset();
          d3d_tex_width_ = 0;
          d3d_tex_height_ = 0;
          for (const auto& candidate : candidates) {
            last_hr = Configure(activations[i], candidate.width,
                                candidate.height, nullptr);
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
          LOG_INFO("Media Foundation hardware AV1 encoder{}: {}",
                   using_d3d11_ ? " (D3D11)" : "", utf8);
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

  HRESULT ReadFrame(IMFSample* sample, const RawFrame& raw,
                    std::vector<EncodedFrame>& frames) {
    output_bytes_.clear();
    HRESULT hr = wmf::ReadBytes(sample, output_bytes_);
    if (FAILED(hr)) return hr;
    auto& bytes = output_bytes_;
    if (bytes.empty()) return MF_E_INVALID_STREAM_DATA;

    UINT32 clean_point = 0;
    const bool sample_clean =
        SUCCEEDED(sample->GetUINT32(MFSampleExtension_CleanPoint,
                                    &clean_point)) &&
        clean_point != 0;
    const auto processed = ProcessEncodedAv1Bitstream(
        bytes, sequence_header_, sample_clean, encode_mode_, source_width_,
        source_height_,
        [this](std::vector<uint8_t>& out_seq_hdr) {
          ComPtr<IMFMediaType> type;
          UINT32 size = 0;
          if (SUCCEEDED(encoder_->GetOutputCurrentType(output_id_, &type)) &&
              SUCCEEDED(type->GetBlobSize(MF_MT_MPEG_SEQUENCE_HEADER, &size)) &&
              size > 0) {
            std::vector<uint8_t> blob(size);
            if (SUCCEEDED(type->GetBlob(MF_MT_MPEG_SEQUENCE_HEADER, blob.data(),
                                        size, nullptr))) {
              CacheSequenceHeaderFromBytes(blob.data(), blob.size(),
                                           out_seq_hdr);
            }
          }
        },
        "WMF AV1");
    if (!processed.valid) return MF_E_INVALID_STREAM_DATA;

    const uint32_t out_w = static_cast<uint32_t>(VisibleWidth());
    const uint32_t out_h = static_cast<uint32_t>(VisibleHeight());
    PushEncodedAv1Frame(bytes, out_w, out_h, processed.is_keyframe,
                        raw.CapturedTimestamp(), clock_->CurrentTime(), frames);
    if (processed.is_keyframe) {
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
    const auto encode_start = std::chrono::steady_clock::now();
    auto encode_submitted = encode_start;
    bool submit_recorded = false;
    double wait_need_input_ms = 0.0;
    double process_input_ms = 0.0;
    const MiniRtcNativeVideoFrame* native = raw.NativeFrame();
    const bool is_d3d11_input =
        native && native->type == MiniRtcNativeVideoFrameD3d11Nv12 &&
        static_cast<ID3D11Device*>(native->payload.d3d11_nv12.d3d11_device) !=
            d3d11_failed_device_;
    if (is_d3d11_input) {
      auto* dev =
          static_cast<ID3D11Device*>(native->payload.d3d11_nv12.d3d11_device);
      const int tex_w =
          static_cast<int>(native->payload.d3d11_nv12.texture_width);
      const int tex_h =
          static_cast<int>(native->payload.d3d11_nv12.texture_height);
      const bool size_changed = static_cast<int>(raw.Width()) != source_width_ ||
                                static_cast<int>(raw.Height()) != source_height_;
      const bool switched_d3d11 = !using_d3d11_;
      const bool dev_changed = dev != d3d_device_.Get();
      const bool tex_changed = tex_w != d3d_tex_width_ || tex_h != d3d_tex_height_;
      if (size_changed || switched_d3d11 || dev_changed || tex_changed ||
          reconfigure_) {
        if (sequence_ > 0) {
          LOG_INFO(
              "Reinitializing WMF AV1 encoder (D3D11): size_chg={} "
              "to_d3d11={} dev_chg={} tex_chg={} reconfig={} {}x{}->{}x{}",
              size_changed, switched_d3d11, dev_changed, tex_changed,
              reconfigure_, source_width_, source_height_, raw.Width(),
              raw.Height());
        }
        source_width_ = static_cast<int>(raw.Width());
        source_height_ = static_cast<int>(raw.Height());
        config_.init_width = source_width_;
        config_.init_height = source_height_;
        d3d_device_ = dev;
        d3d_tex_width_ = tex_w;
        d3d_tex_height_ = tex_h;
        HRESULT hr = Initialize();
        if (FAILED(hr)) return hr;
      }
    } else {
      const bool size_changed = static_cast<int>(raw.Width()) != source_width_ ||
                                static_cast<int>(raw.Height()) != source_height_;
      const bool switched_cpu = using_d3d11_;
      if (size_changed || switched_cpu || reconfigure_) {
        if (sequence_ > 0) {
          LOG_INFO(
              "Reinitializing WMF AV1 encoder (CPU): size_chg={} to_cpu={} "
              "reconfig={} {}x{}->{}x{}",
              size_changed, switched_cpu, reconfigure_, source_width_,
              source_height_, raw.Width(), raw.Height());
        }
        source_width_ = static_cast<int>(raw.Width());
        source_height_ = static_cast<int>(raw.Height());
        config_.init_width = source_width_;
        config_.init_height = source_height_;
        d3d_device_.Reset();
        d3d_tex_width_ = 0;
        d3d_tex_height_ = 0;
        HRESULT hr = Initialize();
        if (FAILED(hr)) return hr;
      }
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
    HRESULT hr = S_OK;
    if (is_d3d11_input && using_d3d11_) {
      auto* tex =
          static_cast<ID3D11Texture2D*>(native->payload.d3d11_nv12.texture);
      ComPtr<IMFMediaBuffer> dxgi_buffer;
      hr = GetOrCreateDxgiBuffer(
          tex, native->payload.d3d11_nv12.subresource_index, dxgi_buffer);
      if (FAILED(hr)) return hr;
      hr = PrepareInputSample(dxgi_buffer.Get(), next_time_, duration, input);
      if (FAILED(hr)) return hr;
    } else {
      const uint8_t* src_nv12 = ResolveCpuNv12(raw, cpu_materialize_buf_);
      if (!src_nv12) return E_INVALIDARG;
      size_t input_size = 0;
      if (!GetNv12FrameSize(encode_width_, encode_height_, &input_size)) {
        return E_INVALIDARG;
      }
      if (!reusable_cpu_buffer_ || reusable_cpu_buffer_size_ < input_size) {
        reusable_cpu_buffer_.Reset();
        hr = MFCreateMemoryBuffer(static_cast<DWORD>(input_size),
                                  &reusable_cpu_buffer_);
        if (FAILED(hr)) return hr;
        reusable_cpu_buffer_size_ = static_cast<DWORD>(input_size);
      }
      BYTE* dst = nullptr;
      hr = reusable_cpu_buffer_->Lock(&dst, nullptr, nullptr);
      if (FAILED(hr)) return hr;
      uint8_t* dst_y = dst;
      uint8_t* dst_uv =
          dst_y + static_cast<size_t>(encode_width_) * encode_height_;
      const bool prepared = PrepareNv12ToStridedPlanes(
          src_nv12, source_width_, source_height_, dst_y, encode_width_,
          dst_uv, encode_width_, encode_width_, encode_height_, encode_mode_,
          &scaler_scratch_);
      reusable_cpu_buffer_->Unlock();
      if (!prepared) return E_FAIL;
      hr = reusable_cpu_buffer_->SetCurrentLength(
          static_cast<DWORD>(input_size));
      if (FAILED(hr)) return hr;
      hr = PrepareInputSample(reusable_cpu_buffer_.Get(), next_time_, duration,
                              input);
      if (FAILED(hr)) return hr;
    }
    struct SampleBufferScope {
      IMFSample* sample = nullptr;
      ~SampleBufferScope() {
        if (sample) sample->RemoveAllBuffers();
      }
    } sample_scope{input.Get()};
    next_time_ += duration;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    bool submitted = false;
    while (std::chrono::steady_clock::now() < deadline) {
      if (!submitted && (!async_ || input_credit_ > 0)) {
        const auto pi_start = std::chrono::steady_clock::now();
        hr = encoder_->ProcessInput(input_id_, input.Get(), 0);
        const auto pi_end = std::chrono::steady_clock::now();
        process_input_ms +=
            std::chrono::duration<double, std::milli>(pi_end - pi_start).count();
        if (SUCCEEDED(hr)) {
          submitted = true;
          encode_submitted = pi_end;
          submit_recorded = true;
          if (async_) --input_credit_;
        } else if (hr != MF_E_NOTACCEPTING) {
          return hr;
        }
      }
      bool have_output = !async_;
      if (async_) {
        ComPtr<IMFMediaEvent> event;
        const DWORD wait_flag =
            (!submitted && input_credit_ > 0) ? MF_EVENT_FLAG_NO_WAIT : 0;
        const auto ev_start = std::chrono::steady_clock::now();
        hr = events_->GetEvent(wait_flag, &event);
        const auto ev_end = std::chrono::steady_clock::now();
        if (!submitted) {
          wait_need_input_ms +=
              std::chrono::duration<double, std::milli>(ev_end - ev_start)
                  .count();
        }
        if (hr != MF_E_NO_EVENTS_AVAILABLE) {
          if (FAILED(hr)) return hr;
          HRESULT status = S_OK;
          hr = event->GetStatus(&status);
          if (FAILED(hr)) return hr;
          if (FAILED(status)) return status;
          MediaEventType type = MEUnknown;
          hr = event->GetType(&type);
          if (FAILED(hr)) return hr;
          if (type == METransformNeedInput) {
            ++input_credit_;
            if (!submitted && input_credit_ > 0) {
              const auto pi_start = std::chrono::steady_clock::now();
              hr = encoder_->ProcessInput(input_id_, input.Get(), 0);
              const auto pi_end = std::chrono::steady_clock::now();
              process_input_ms +=
                  std::chrono::duration<double, std::milli>(pi_end - pi_start)
                      .count();
              if (SUCCEEDED(hr)) {
                submitted = true;
                encode_submitted = pi_end;
                submit_recorded = true;
                --input_credit_;
              } else if (hr != MF_E_NOTACCEPTING) {
                return hr;
              }
            }
          }
          if (type == METransformHaveOutput) have_output = true;
        }
      }
      if (have_output) {
        ComPtr<IMFSample> output;
        hr = wmf::ReadOutput(encoder_.Get(), output_id_,
                             reusable_output_sample_, output);
        if (hr == MF_E_TRANSFORM_STREAM_CHANGE) {
          reusable_output_sample_.Reset();
          ComPtr<IMFMediaType> type;
          hr = encoder_->GetOutputAvailableType(output_id_, 0, &type);
          if (FAILED(hr)) return hr;
          hr = encoder_->SetOutputType(output_id_, type.Get(), 0);
          if (FAILED(hr)) return hr;
          if (async_) continue;
          hr = wmf::ReadOutput(encoder_.Get(), output_id_,
                               reusable_output_sample_, output);
        }
        if (FAILED(hr) && hr != MF_E_TRANSFORM_NEED_MORE_INPUT) return hr;
        if (SUCCEEDED(hr) && output) {
          hr = ReadFrame(output.Get(), raw, frames);
          if (FAILED(hr)) return hr;
          ++sequence_;
          const auto encode_end = std::chrono::steady_clock::now();
          const double total_ms =
              std::chrono::duration<double, std::milli>(encode_end - encode_start)
                  .count();
          const double hw_ms =
              submit_recorded
                  ? std::chrono::duration<double, std::milli>(encode_end -
                                                              encode_submitted)
                        .count()
                  : 0.0;
          const size_t encoded_bytes =
              frames.empty() ? 0 : frames.back().Size();
          const bool is_key =
              !frames.empty() &&
              frames.back().FrameType() == VideoFrameType::kVideoFrameKey;
          stats_.Record("WMF AV1", encode_end, encode_width_, encode_height_,
                        using_d3d11_, total_ms, wait_need_input_ms,
                        process_input_ms, hw_ms, encoded_bytes, is_key);
          return S_OK;
        }
      }
      if (!async_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    return HRESULT_FROM_WIN32(WAIT_TIMEOUT);
  }

  wmf::CodecThread worker_;
  wmf::CallbackThread callback_worker_;
  std::shared_ptr<SystemClock> clock_;
  MediaCodecConfig config_;
  ComPtr<IMFTransform> encoder_;
  ComPtr<IMFMediaEventGenerator> events_;
  ComPtr<ICodecAPI> api_;
  ComPtr<ID3D11Device> d3d_device_;
  ID3D11Device* d3d11_failed_device_ = nullptr;
  ComPtr<IMFDXGIDeviceManager> dxgi_manager_;
  UINT dxgi_reset_token_ = 0;
  bool using_d3d11_ = false;
  int d3d_tex_width_ = 0, d3d_tex_height_ = 0;
  struct CachedDxgiBuffer {
    ID3D11Texture2D* texture = nullptr;
    uint32_t subresource = 0;
    ComPtr<IMFMediaBuffer> buffer;
  };
  std::vector<CachedDxgiBuffer> dxgi_buffer_cache_;
  ComPtr<IMFSample> reusable_input_sample_;
  wmf::ReusableOutputSample reusable_output_sample_;
  ComPtr<IMFMediaBuffer> reusable_cpu_buffer_;
  DWORD reusable_cpu_buffer_size_ = 0;
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
  std::vector<uint8_t> output_bytes_;
  std::vector<uint8_t> cpu_materialize_buf_;
  std::vector<uint8_t> scaler_scratch_;
  std::unique_ptr<SvtAv1Encoder> fallback_;
  EncodeStatsTracker stats_;
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
  impl_->callback_worker_.Flush();
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
    impl_->d3d_device_.Reset();
    impl_->d3d_tex_width_ = 0;
    impl_->d3d_tex_height_ = 0;
    impl_->initialized_ = SUCCEEDED(impl_->Initialize());
    return impl_->initialized_ ? 0 : -1;
  });
}

bool WmfAv1Encoder::SupportsNativeFrameInput(
    MiniRtcNativeVideoFrameType frame_type) const {
  return frame_type == MiniRtcNativeVideoFrameD3d11Nv12 ||
         frame_type == MiniRtcNativeVideoFrameCpuNv12;
}

int WmfAv1Encoder::Encode(const RawFrame& raw,
                          std::function<int(const EncodedFrame&)> callback) {
  size_t size = 0;
  if (raw.Width() == 0 || raw.Height() == 0 || raw.Width() > 8192 ||
      raw.Height() > 8192 ||
      !GetNv12FrameSize(raw.Width(), raw.Height(), &size) ||
      (!raw.NativeFrame() && (!raw.Buffer() || raw.Size() < size)))
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
      impl_->d3d_device_.Reset();
      impl_->d3d_tex_width_ = 0;
      impl_->d3d_tex_height_ = 0;
      impl_->fallback_ = std::make_unique<SvtAv1Encoder>(impl_->clock_);
      if (impl_->fallback_->Init(impl_->config_) != 0) {
        impl_->fallback_.reset();
        return -1;
      }
    }
    RawFrame fallback_raw = raw;
    if (fallback_raw.NativeFrame() && !fallback_raw.MaterializeNativeFrame()) {
      return -1;
    }
    return impl_->fallback_->Encode(
        fallback_raw, [&](const EncodedFrame& frame) {
          frames.push_back(frame);
          return 0;
        });
  });
  if (result == 0 && !frames.empty() && callback) {
    impl_->callback_worker_.Post(
        [frames = std::move(frames), cb = std::move(callback)]() mutable {
          for (const auto& frame : frames) {
            cb(frame);
          }
        });
  }
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
    const int min_supported = std::min(10000, impl_->config_.max_bitrate);
    bitrate = std::max(bitrate, min_supported);
    if (impl_->fallback_) {
      const int result = impl_->fallback_->SetTargetBitrate(bitrate);
      if (result == 0) impl_->config_.init_bitrate = bitrate;
      return result;
    }
    if (!impl_->api_) return -1;
    const int applied = impl_->config_.init_bitrate;
    if (bitrate == applied) return 0;
    const bool at_bound =
        bitrate == min_supported || bitrate == impl_->config_.max_bitrate;
    // Ignore minor BWE/FEC fluctuations (<~14.3%, e.g. 0%<->10% FEC toggles)
    // unless hitting a hard bound; AMD's MFT rate controller can stall >100 ms
    // across frequent MeanBitRate resets.
    if (!at_bound && applied > 0 &&
        std::llabs(static_cast<int64_t>(bitrate) - applied) * 7 < applied) {
      return 0;
    }
    const HRESULT hr =
        SetUint(impl_->api_.Get(), CODECAPI_AVEncCommonMeanBitRate, bitrate);
    if (FAILED(hr)) {
      LOG_WARN("WMF AV1 SetUint MeanBitRate({}) failed: 0x{:08x}", bitrate,
               static_cast<uint32_t>(hr));
    }
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
