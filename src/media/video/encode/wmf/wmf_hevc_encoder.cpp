#include "wmf_hevc_encoder.h"

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

#include "log.h"
#include "wmf/wmf_utils.h"

namespace minirtc {
namespace {
using wmf::ComPtr;

// MFVideoFormat_HEVC ('HEVC') defined explicitly for broad Windows SDK compat.
constexpr GUID kVideoFormatHevc = {
    0x43564548,
    0x0000,
    0x0010,
    {0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71}};

constexpr uint8_t kHevcNaluVps = 32;
constexpr uint8_t kHevcNaluSps = 33;
constexpr uint8_t kHevcNaluPps = 34;
constexpr uint8_t kHevcNaluAud = 35;

HRESULT SetUint(ICodecAPI* api, const GUID& key, ULONG value) {
  VARIANT variant{};
  variant.vt = VT_UI4;
  variant.ulVal = value;
  return api->SetValue(&key, &variant);
}

HRESULT EnumerateEncoders(IMFActivate*** activates, UINT32* count) {
  MFT_REGISTER_TYPE_INFO input{MFMediaType_Video, MFVideoFormat_NV12};
  MFT_REGISTER_TYPE_INFO output{MFMediaType_Video, kVideoFormatHevc};
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

}  // namespace

class WmfHevcEncoder::Impl {
 public:
  explicit Impl(std::shared_ptr<SystemClock> clock)
      : clock_(std::move(clock)) {}
  ~Impl() {
    callback_worker_.Flush();
    worker_.Call([this] {
      Reset();
      d3d_device_.Reset();
      d3d_context_.Reset();
      d3d_crop_tex_.Reset();
      d3d_crop_w_ = 0;
      d3d_crop_h_ = 0;
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
    vps_.clear();
    sps_.clear();
    pps_.clear();
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

  HRESULT CopyToExactSizeTexture(ID3D11Texture2D* src_tex, uint32_t subresource,
                                 int width, int height,
                                 ID3D11Texture2D** out_tex) {
    if (!d3d_device_ || !src_tex || width <= 0 || height <= 0 || !out_tex) {
      return E_INVALIDARG;
    }
    if (!d3d_context_) {
      d3d_device_->GetImmediateContext(&d3d_context_);
      if (!d3d_context_) return E_FAIL;
    }
    if (!d3d_crop_tex_ || d3d_crop_w_ != width || d3d_crop_h_ != height) {
      d3d_crop_tex_.Reset();
      dxgi_buffer_cache_.clear();
      D3D11_TEXTURE2D_DESC desc{};
      desc.Width = static_cast<UINT>(width);
      desc.Height = static_cast<UINT>(height);
      desc.MipLevels = 1;
      desc.ArraySize = 1;
      desc.Format = DXGI_FORMAT_NV12;
      desc.SampleDesc.Count = 1;
      desc.Usage = D3D11_USAGE_DEFAULT;
      desc.BindFlags = D3D11_BIND_RENDER_TARGET;
      HRESULT hr = d3d_device_->CreateTexture2D(&desc, nullptr, &d3d_crop_tex_);
      if (FAILED(hr)) {
        desc.BindFlags = 0;
        hr = d3d_device_->CreateTexture2D(&desc, nullptr, &d3d_crop_tex_);
        if (FAILED(hr)) return hr;
      }
      d3d_crop_w_ = width;
      d3d_crop_h_ = height;
    }
    D3D11_BOX box{};
    box.left = 0;
    box.top = 0;
    box.front = 0;
    box.right = static_cast<UINT>(width);
    box.bottom = static_cast<UINT>(height);
    box.back = 1;
    d3d_context_->CopySubresourceRegion(d3d_crop_tex_.Get(), 0, 0, 0, 0,
                                        src_tex, subresource, &box);
    *out_tex = d3d_crop_tex_.Get();
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
      SetUint(api_.Get(), CODECAPI_AVEncMPVGOPSize,
              static_cast<ULONG>(config_.key_frame_interval));
      SetUint(api_.Get(), CODECAPI_AVEncCommonQualityVsSpeed,
              prioritize_encoding_speed_ ? 0u : 15u);
    }

    ComPtr<IMFMediaType> output, input;
    hr = MFCreateMediaType(&output);
    if (FAILED(hr)) return hr;
    output->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    output->SetGUID(MF_MT_SUBTYPE, kVideoFormatHevc);
    output->SetUINT32(MF_MT_AVG_BITRATE,
                      static_cast<UINT32>(config_.init_bitrate));
    output->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
    MFSetAttributeSize(output.Get(), MF_MT_FRAME_SIZE,
                       static_cast<UINT32>(width), static_cast<UINT32>(height));
    MFSetAttributeRatio(output.Get(), MF_MT_FRAME_RATE,
                        static_cast<UINT32>(config_.max_frame_rate), 1);
    MFSetAttributeRatio(output.Get(), MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    hr = encoder_->SetOutputType(output_id_, output.Get(), 0);
    if (FAILED(hr)) {
      LOG_WARN("WMF HEVC SetOutputType failed (0x{:08x}); retrying Main profile",
               static_cast<uint32_t>(hr));
      output->SetUINT32(MF_MT_MPEG2_PROFILE, 1);
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
      input->SetUINT32(MF_MT_DEFAULT_STRIDE, static_cast<UINT32>(width));
    }
    hr = encoder_->SetInputType(input_id_, input.Get(), 0);
    if (FAILED(hr) && !using_d3d11_) {
      LOG_WARN("WMF HEVC SetInputType failed (0x{:08x}); retrying without stride",
               static_cast<uint32_t>(hr));
      input->DeleteItem(MF_MT_DEFAULT_STRIDE);
      hr = encoder_->SetInputType(input_id_, input.Get(), 0);
    }
    if (FAILED(hr)) return hr;

    if (api_) {
      SetUint(api_.Get(), CODECAPI_AVEncCommonMeanBitRate,
              static_cast<ULONG>(config_.init_bitrate));
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
    hr = MF_E_TOPO_CODEC_NOT_FOUND;
    for (UINT32 i = 0; i < count; ++i) {
      if (!encoder_) {
        HRESULT last_hr = MF_E_TOPO_CODEC_NOT_FOUND;
        if (d3d_device_) {
          last_hr = Configure(activations[i], source_width_, source_height_,
                              d3d_device_.Get());
          if (SUCCEEDED(last_hr)) {
            d3d11_failed_device_ = nullptr;
          } else {
            LOG_WARN(
                "Hardware HEVC MFT D3D11 config {}x{} failed (0x{:08x}); "
                "falling back to CPU buffer input",
                source_width_, source_height_, static_cast<uint32_t>(last_hr));
            d3d11_failed_device_ = d3d_device_.Get();
            Reset();
            activations[i]->ShutdownObject();
          }
        }
        if (!encoder_) {
          d3d_device_.Reset();
          d3d_context_.Reset();
          d3d_crop_tex_.Reset();
          d3d_crop_w_ = 0;
          d3d_crop_h_ = 0;
          last_hr =
              Configure(activations[i], source_width_, source_height_, nullptr);
          if (FAILED(last_hr)) {
            Reset();
            activations[i]->ShutdownObject();
          }
        }
        hr = last_hr;
        if (FAILED(hr)) {
          LOG_WARN("Hardware HEVC MFT initialization failed: 0x{:08x}",
                   static_cast<uint32_t>(hr));
        } else {
          WCHAR name[256]{};
          activations[i]->GetString(MFT_FRIENDLY_NAME_Attribute, name, 256,
                                    nullptr);
          char utf8[1024]{};
          WideCharToMultiByte(CP_UTF8, 0, name, -1, utf8, sizeof(utf8), nullptr,
                              nullptr);
          LOG_INFO("Media Foundation hardware HEVC encoder{} ({}x{}): {}",
                   using_d3d11_ ? " (D3D11)" : "", source_width_,
                   source_height_, utf8);
          activations[i]->DetachObject();
        }
      }
      activations[i]->Release();
    }
    CoTaskMemFree(activations);
    reconfigure_ = false;
    return encoder_ ? S_OK : hr;
  }

  void CacheParameterSets(const uint8_t* data, size_t size) {
    static constexpr uint8_t kStartCode[4] = {0, 0, 0, 1};
    for (const auto& nalu : wmf::AnnexBHevcNalus(data, size)) {
      std::vector<uint8_t>* target = nullptr;
      if (nalu.type == kHevcNaluVps) {
        target = &vps_;
      } else if (nalu.type == kHevcNaluSps) {
        target = &sps_;
      } else if (nalu.type == kHevcNaluPps) {
        target = &pps_;
      }
      if (target) {
        target->clear();
        target->reserve(4 + nalu.size);
        target->insert(target->end(), kStartCode, kStartCode + 4);
        target->insert(target->end(), data + nalu.offset,
                       data + nalu.offset + nalu.size);
      }
    }
  }

  HRESULT ReadFrame(IMFSample* sample, const RawFrame& raw,
                    std::vector<EncodedFrame>& frames) {
    output_bytes_.clear();
    HRESULT hr = wmf::ReadBytes(sample, output_bytes_);
    if (FAILED(hr)) return hr;
    auto& bytes = output_bytes_;
    if (bytes.empty()) return MF_E_INVALID_STREAM_DATA;

    const auto nalus = wmf::AnnexBHevcNalus(bytes.data(), bytes.size());
    if (nalus.empty()) {
      LOG_WARN("WMF HEVC output is not a valid Annex-B stream (size={})",
               bytes.size());
      return MF_E_INVALID_STREAM_DATA;
    }

    CacheParameterSets(bytes.data(), bytes.size());

    bool has_vps = false;
    bool has_sps = false;
    bool has_pps = false;
    bool nalu_is_key = false;
    for (const auto& nalu : nalus) {
      if (nalu.type == kHevcNaluVps) has_vps = true;
      else if (nalu.type == kHevcNaluSps) has_sps = true;
      else if (nalu.type == kHevcNaluPps) has_pps = true;
      else if (nalu.type == 19 || nalu.type == 20 || nalu.type == 21) {
        nalu_is_key = true;
      }
    }

    UINT32 clean_point = 0;
    const bool sample_clean =
        SUCCEEDED(
            sample->GetUINT32(MFSampleExtension_CleanPoint, &clean_point)) &&
        clean_point != 0;
    const bool key = sample_clean || nalu_is_key;

    if (key && (!has_vps || !has_sps || !has_pps)) {
      if (vps_.empty() || sps_.empty() || pps_.empty()) {
        ComPtr<IMFMediaType> type;
        UINT32 blob_size = 0;
        if (SUCCEEDED(encoder_->GetOutputCurrentType(output_id_, &type)) &&
            SUCCEEDED(type->GetBlobSize(MF_MT_MPEG_SEQUENCE_HEADER,
                                        &blob_size)) &&
            blob_size > 0) {
          std::vector<uint8_t> blob(blob_size);
          if (SUCCEEDED(type->GetBlob(MF_MT_MPEG_SEQUENCE_HEADER, blob.data(),
                                      blob_size, nullptr))) {
            CacheParameterSets(blob.data(), blob.size());
          }
        }
      }
      if (!vps_.empty() && !sps_.empty() && !pps_.empty()) {
        std::vector<uint8_t> headers;
        headers.reserve(vps_.size() + sps_.size() + pps_.size());
        headers.insert(headers.end(), vps_.begin(), vps_.end());
        headers.insert(headers.end(), sps_.begin(), sps_.end());
        headers.insert(headers.end(), pps_.begin(), pps_.end());
        // Insert right after a leading AUD if present, otherwise at byte 0.
        size_t insert_offset = 0;
        if (!nalus.empty() && nalus[0].type == kHevcNaluAud) {
          insert_offset = nalus[0].offset + nalus[0].size;
        }
        bytes.insert(bytes.begin() + static_cast<ptrdiff_t>(insert_offset),
                     headers.begin(), headers.end());
      }
    }

    const uint32_t out_w = static_cast<uint32_t>(source_width_);
    const uint32_t out_h = static_cast<uint32_t>(source_height_);
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

  const uint8_t* ResolveCpuNv12(const RawFrame& raw) {
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
    cpu_materialize_buf_.resize(raw_size);
    if (!native->copy_to_nv12 ||
        native->copy_to_nv12(native->owner, cpu_materialize_buf_.data(),
                             raw_size) != 0) {
      return nullptr;
    }
    return cpu_materialize_buf_.data();
  }

  HRESULT EncodeHardware(const RawFrame& raw,
                         std::vector<EncodedFrame>& frames) {
    const auto encode_start = std::chrono::steady_clock::now();
    auto encode_submitted = encode_start;
    bool submit_recorded = false;
    const MiniRtcNativeVideoFrame* native = raw.NativeFrame();
    const bool is_d3d11_input =
        native && native->type == MiniRtcNativeVideoFrameD3d11Nv12 &&
        static_cast<ID3D11Device*>(native->payload.d3d11_nv12.d3d11_device) !=
            d3d11_failed_device_;

    if (is_d3d11_input) {
      auto* dev =
          static_cast<ID3D11Device*>(native->payload.d3d11_nv12.d3d11_device);
      const bool size_changed =
          static_cast<int>(raw.Width()) != source_width_ ||
          static_cast<int>(raw.Height()) != source_height_;
      const bool switched_d3d11 = !using_d3d11_;
      const bool dev_changed = dev != d3d_device_.Get();
      if (size_changed || switched_d3d11 || dev_changed || reconfigure_) {
        if (sequence_ > 0) {
          LOG_INFO(
              "Reinitializing WMF HEVC encoder (D3D11): size_chg={} "
              "to_d3d11={} dev_chg={} reconfig={} {}x{}->{}x{}",
              size_changed, switched_d3d11, dev_changed, reconfigure_,
              source_width_, source_height_, raw.Width(), raw.Height());
        }
        source_width_ = static_cast<int>(raw.Width());
        source_height_ = static_cast<int>(raw.Height());
        config_.init_width = source_width_;
        config_.init_height = source_height_;
        d3d_device_ = dev;
        d3d_context_.Reset();
        d3d_crop_tex_.Reset();
        d3d_crop_w_ = 0;
        d3d_crop_h_ = 0;
        if (size_changed || dev_changed) {
          need_d3d11_crop_copy_ = false;
        }
        HRESULT hr = Initialize();
        if (FAILED(hr)) return hr;
      }
    } else {
      const bool size_changed =
          static_cast<int>(raw.Width()) != source_width_ ||
          static_cast<int>(raw.Height()) != source_height_;
      const bool switched_cpu = using_d3d11_;
      if (size_changed || switched_cpu || reconfigure_) {
        if (sequence_ > 0) {
          LOG_INFO(
              "Reinitializing WMF HEVC encoder (CPU): size_chg={} to_cpu={} "
              "reconfig={} {}x{}->{}x{}",
              size_changed, switched_cpu, reconfigure_, source_width_,
              source_height_, raw.Width(), raw.Height());
        }
        source_width_ = static_cast<int>(raw.Width());
        source_height_ = static_cast<int>(raw.Height());
        config_.init_width = source_width_;
        config_.init_height = source_height_;
        d3d_device_.Reset();
        d3d_context_.Reset();
        d3d_crop_tex_.Reset();
        d3d_crop_w_ = 0;
        d3d_crop_h_ = 0;
        HRESULT hr = Initialize();
        if (FAILED(hr)) return hr;
      }
    }

    if (!encoder_) return MF_E_NOT_INITIALIZED;
    if (sequence_ % static_cast<uint64_t>(config_.key_frame_interval) == 0) {
      force_idr_ = true;
    }
    if (force_idr_ && api_) {
      SetUint(api_.Get(), CODECAPI_AVEncVideoForceKeyFrame, 1);
    }

    ComPtr<IMFSample> input;
    const LONGLONG duration = 10'000'000LL / config_.max_frame_rate;
    HRESULT hr = S_OK;
    bool tex_larger_than_visible = false;
    if (is_d3d11_input && using_d3d11_) {
      auto* tex =
          static_cast<ID3D11Texture2D*>(native->payload.d3d11_nv12.texture);
      uint32_t subresource = native->payload.d3d11_nv12.subresource_index;
      tex_larger_than_visible =
          static_cast<int>(native->payload.d3d11_nv12.texture_width) !=
              source_width_ ||
          static_cast<int>(native->payload.d3d11_nv12.texture_height) !=
              source_height_;
      if (tex_larger_than_visible && need_d3d11_crop_copy_) {
        ID3D11Texture2D* exact_tex = nullptr;
        hr = CopyToExactSizeTexture(tex, subresource, source_width_,
                                    source_height_, &exact_tex);
        if (FAILED(hr)) return hr;
        tex = exact_tex;
        subresource = 0;
      }
      ComPtr<IMFMediaBuffer> dxgi_buffer;
      hr = GetOrCreateDxgiBuffer(tex, subresource, dxgi_buffer);
      if (FAILED(hr)) return hr;
      hr = PrepareInputSample(dxgi_buffer.Get(), next_time_, duration, input);
      if (FAILED(hr)) return hr;
    } else {
      const uint8_t* src_nv12 = ResolveCpuNv12(raw);
      if (!src_nv12) return E_INVALIDARG;
      size_t input_size = 0;
      if (!GetNv12FrameSize(source_width_, source_height_, &input_size)) {
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
      std::memcpy(dst, src_nv12, input_size);
      reusable_cpu_buffer_->Unlock();
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
        hr = encoder_->ProcessInput(input_id_, input.Get(), 0);
        if (SUCCEEDED(hr)) {
          submitted = true;
          encode_submitted = std::chrono::steady_clock::now();
          submit_recorded = true;
          if (async_) --input_credit_;
        } else if (hr != MF_E_NOTACCEPTING) {
          if (is_d3d11_input && using_d3d11_ && tex_larger_than_visible &&
              !need_d3d11_crop_copy_) {
            LOG_WARN(
                "WMF HEVC ProcessInput rejected padded D3D11 texture "
                "(0x{:08x}); switching to GPU exact-size crop copy",
                static_cast<uint32_t>(hr));
            need_d3d11_crop_copy_ = true;
          }
          return hr;
        }
      }
      bool have_output = !async_;
      if (async_) {
        ComPtr<IMFMediaEvent> event;
        const DWORD wait_flag =
            (!submitted && input_credit_ > 0) ? MF_EVENT_FLAG_NO_WAIT : 0;
        hr = events_->GetEvent(wait_flag, &event);
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
              hr = encoder_->ProcessInput(input_id_, input.Get(), 0);
              if (SUCCEEDED(hr)) {
                submitted = true;
                encode_submitted = std::chrono::steady_clock::now();
                submit_recorded = true;
                --input_credit_;
              } else if (hr != MF_E_NOTACCEPTING) {
                if (is_d3d11_input && using_d3d11_ && tex_larger_than_visible &&
                    !need_d3d11_crop_copy_) {
                  need_d3d11_crop_copy_ = true;
                }
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
              std::chrono::duration<double, std::milli>(encode_end -
                                                        encode_start)
                  .count();
          const double submit_ms =
              submit_recorded
                  ? std::chrono::duration<double, std::milli>(encode_submitted -
                                                              encode_start)
                        .count()
                  : total_ms;
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
          RecordEncodeStats(encode_end, total_ms, submit_ms, hw_ms,
                            encoded_bytes, is_key);
          return S_OK;
        }
      }
      if (!async_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    return HRESULT_FROM_WIN32(WAIT_TIMEOUT);
  }

  static double Percentile(std::vector<double>& samples, double pct) {
    if (samples.empty()) return 0.0;
    const size_t idx = std::min(
        samples.size() - 1,
        static_cast<size_t>(pct * static_cast<double>(samples.size() - 1)));
    std::nth_element(samples.begin(), samples.begin() + idx, samples.end());
    return samples[idx];
  }

  void RecordEncodeStats(std::chrono::steady_clock::time_point now,
                         double total_ms, double submit_ms, double hw_ms,
                         size_t bytes, bool keyframe) {
    if (stat_window_start_.time_since_epoch().count() == 0) {
      stat_window_start_ = now;
    }
    stat_total_ms_.push_back(total_ms);
    stat_submit_ms_.push_back(submit_ms);
    stat_hw_ms_.push_back(hw_ms);
    stat_total_bytes_ += bytes;
    if (keyframe) ++stat_keyframes_;
    const double elapsed_s =
        std::chrono::duration<double>(now - stat_window_start_).count();
    if (elapsed_s < 5.0 || stat_total_ms_.empty()) return;

    const size_t n = stat_total_ms_.size();
    const double enc_p50 = Percentile(stat_total_ms_, 0.50);
    const double enc_p95 = Percentile(stat_total_ms_, 0.95);
    const double enc_max =
        *std::max_element(stat_total_ms_.begin(), stat_total_ms_.end());
    const double sub_p50 = Percentile(stat_submit_ms_, 0.50);
    const double sub_p95 = Percentile(stat_submit_ms_, 0.95);
    const double hw_p50 = Percentile(stat_hw_ms_, 0.50);
    const double hw_p95 = Percentile(stat_hw_ms_, 0.95);
    const double avg_kb = static_cast<double>(stat_total_bytes_) /
                          (static_cast<double>(n) * 1024.0);
    LOG_INFO(
        "WMF HEVC stats ({:.1f}s): {}x{} d3d11={} frames={} fps={:.1f} kf={} "
        "enc_ms[p50={:.2f} p95={:.2f} max={:.2f}] "
        "submit_ms[p50={:.2f} p95={:.2f}] hw_ms[p50={:.2f} p95={:.2f}] "
        "avg_kb={:.1f}",
        elapsed_s, source_width_, source_height_, using_d3d11_, n,
        static_cast<double>(n) / elapsed_s, stat_keyframes_, enc_p50, enc_p95,
        enc_max, sub_p50, sub_p95, hw_p50, hw_p95, avg_kb);
    stat_window_start_ = now;
    stat_total_ms_.clear();
    stat_submit_ms_.clear();
    stat_hw_ms_.clear();
    stat_total_bytes_ = 0;
    stat_keyframes_ = 0;
  }

  wmf::CodecThread worker_;
  wmf::CallbackThread callback_worker_;
  std::shared_ptr<SystemClock> clock_;
  MediaCodecConfig config_;
  ComPtr<IMFTransform> encoder_;
  ComPtr<IMFMediaEventGenerator> events_;
  ComPtr<ICodecAPI> api_;
  ComPtr<ID3D11Device> d3d_device_;
  ComPtr<ID3D11DeviceContext> d3d_context_;
  ComPtr<ID3D11Texture2D> d3d_crop_tex_;
  int d3d_crop_w_ = 0;
  int d3d_crop_h_ = 0;
  bool need_d3d11_crop_copy_ = false;
  ID3D11Device* d3d11_failed_device_ = nullptr;
  ComPtr<IMFDXGIDeviceManager> dxgi_manager_;
  UINT dxgi_reset_token_ = 0;
  bool using_d3d11_ = false;
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
  bool prioritize_encoding_speed_ = true;
  bool initialized_ = false;
  int source_width_ = 0, source_height_ = 0;
  uint64_t sequence_ = 0;
  LONGLONG next_time_ = 0;
  std::vector<uint8_t> vps_;
  std::vector<uint8_t> sps_;
  std::vector<uint8_t> pps_;
  std::vector<uint8_t> output_bytes_;
  std::vector<uint8_t> cpu_materialize_buf_;
  std::chrono::steady_clock::time_point stat_window_start_;
  std::vector<double> stat_total_ms_;
  std::vector<double> stat_submit_ms_;
  std::vector<double> stat_hw_ms_;
  size_t stat_total_bytes_ = 0;
  uint64_t stat_keyframes_ = 0;
};

WmfHevcEncoder::WmfHevcEncoder(std::shared_ptr<SystemClock> clock)
    : impl_(std::make_unique<Impl>(std::move(clock))) {}
WmfHevcEncoder::~WmfHevcEncoder() = default;

bool WmfHevcEncoder::IsSupported() {
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

int WmfHevcEncoder::Init(const MediaCodecConfig& config) {
  if (!ValidConfig(config)) return -1;
  impl_->callback_worker_.Flush();
  return impl_->worker_.Call([&] {
    impl_->config_ = config;
    impl_->config_.init_bitrate =
        ClampEncoderTargetBitrate(config.init_bitrate, config.max_bitrate);
    if (impl_->config_.average_bitrate <= 0) {
      impl_->config_.average_bitrate = impl_->config_.init_bitrate;
    }
    impl_->source_width_ = impl_->config_.init_width;
    impl_->source_height_ = impl_->config_.init_height;
    impl_->d3d_device_.Reset();
    impl_->d3d_context_.Reset();
    impl_->d3d_crop_tex_.Reset();
    impl_->d3d_crop_w_ = 0;
    impl_->d3d_crop_h_ = 0;
    impl_->need_d3d11_crop_copy_ = false;
    impl_->initialized_ = SUCCEEDED(impl_->Initialize());
    return impl_->initialized_ ? 0 : -1;
  });
}

bool WmfHevcEncoder::SupportsNativeFrameInput(
    MiniRtcNativeVideoFrameType frame_type) const {
  return frame_type == MiniRtcNativeVideoFrameD3d11Nv12 ||
         frame_type == MiniRtcNativeVideoFrameCpuNv12;
}

int WmfHevcEncoder::Encode(const RawFrame& raw,
                           std::function<int(const EncodedFrame&)> callback) {
  size_t size = 0;
  if (raw.Width() == 0 || raw.Height() == 0 || raw.Width() > 8192 ||
      raw.Height() > 8192 ||
      !GetNv12FrameSize(raw.Width(), raw.Height(), &size) ||
      (!raw.NativeFrame() && (!raw.Buffer() || raw.Size() < size))) {
    return -1;
  }
  std::vector<EncodedFrame> frames;
  const int result = impl_->worker_.Call([&] {
    if (!impl_->initialized_) return -1;
    HRESULT hr = impl_->EncodeHardware(raw, frames);
    if (SUCCEEDED(hr)) return 0;
    LOG_WARN("Media Foundation hardware HEVC encoding failed (0x{:08x})",
             static_cast<uint32_t>(hr));
    // If D3D11 crop copy was just enabled or D3D11 failed mid-stream, retry
    // once immediately (either with exact-size GPU crop or CPU input).
    if (impl_->using_d3d11_) {
      if (impl_->need_d3d11_crop_copy_) {
        hr = impl_->EncodeHardware(raw, frames);
        if (SUCCEEDED(hr)) return 0;
      }
      LOG_WARN("Falling back WMF HEVC encoder from D3D11 to CPU buffer input");
      impl_->d3d11_failed_device_ = impl_->d3d_device_.Get();
      impl_->reconfigure_ = true;
      hr = impl_->EncodeHardware(raw, frames);
      if (SUCCEEDED(hr)) return 0;
    }
    return -1;
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

int WmfHevcEncoder::ForceIdr() {
  return impl_->worker_.Call([&] {
    impl_->force_idr_ = true;
    return 0;
  });
}

int WmfHevcEncoder::SetTargetBitrate(int bitrate) {
  if (bitrate <= 0) return -1;
  return impl_->worker_.Call([&] {
    bitrate = ClampEncoderTargetBitrate(bitrate, impl_->config_.max_bitrate);
    const int min_supported = std::min(10000, impl_->config_.max_bitrate);
    bitrate = std::max(bitrate, min_supported);
    if (!impl_->api_) return -1;
    const int applied = impl_->config_.init_bitrate;
    if (bitrate == applied) return 0;
    const bool at_bound =
        bitrate == min_supported || bitrate == impl_->config_.max_bitrate;
    // Ignore minor BWE/FEC fluctuations (<20%) unless hitting a hard bound;
    // AMD's WMF MFT rate controller can stall across frequent MeanBitRate resets.
    if (!at_bound && applied > 0 &&
        std::llabs(static_cast<int64_t>(bitrate) - applied) * 5 < applied) {
      return 0;
    }
    const HRESULT hr = SetUint(impl_->api_.Get(),
                               CODECAPI_AVEncCommonMeanBitRate,
                               static_cast<ULONG>(bitrate));
    if (FAILED(hr)) {
      LOG_WARN("WMF HEVC SetUint MeanBitRate({}) failed: 0x{:08x}", bitrate,
               static_cast<uint32_t>(hr));
    }
    impl_->config_.init_bitrate = bitrate;
    return 0;
  });
}

int WmfHevcEncoder::SetTargetFrameRate(int frame_rate) {
  if (frame_rate <= 0 || frame_rate > 240) return -1;
  return impl_->worker_.Call([&] {
    if (!impl_->encoder_) return -1;
    if (impl_->config_.max_frame_rate != frame_rate) impl_->reconfigure_ = true;
    impl_->config_.max_frame_rate = frame_rate;
    return 0;
  });
}

int WmfHevcEncoder::SetPrioritizeEncodingSpeedOverQuality(
    bool prioritize_speed) {
  return impl_->worker_.Call([&] {
    impl_->prioritize_encoding_speed_ = prioritize_speed;
    if (impl_->api_) {
      SetUint(impl_->api_.Get(), CODECAPI_AVEncCommonQualityVsSpeed,
              prioritize_speed ? 0u : 15u);
    }
    return 0;
  });
}

int WmfHevcEncoder::GetResolution(int* width, int* height) const {
  if (!width || !height) return -1;
  return impl_->worker_.Call([&] {
    if (!impl_->encoder_) return -1;
    *width = impl_->source_width_;
    *height = impl_->source_height_;
    return 0;
  });
}

std::string WmfHevcEncoder::GetEncoderName() const {
  return "Media Foundation (HEVC, HW)";
}

}  // namespace minirtc
