#include "d3d11_hevc_decoder.h"

#include <codecapi.h>
#include <d3d10_1.h>
#include <d3d11.h>
#include <dxgi1_2.h>
#include <mfapi.h>

#include <cstring>
#include <limits>
#include <map>
#include <vector>

#include "log.h"
#include "native_nv12_frame.h"
#include "wmf/wmf_utils.h"

namespace minirtc {
namespace {
using wmf::ComPtr;

// MFVideoFormat_HEVC ('HEVC')
constexpr GUID kVideoFormatHevc = {
    0x43564548,
    0x0000,
    0x0010,
    {0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71}};

// D3D11_DECODER_PROFILE_HEVC_VLD_MAIN = {5B11D51B-2F4C-4452-BCC3-09F2A1160CC0}
constexpr GUID kD3d11DecoderProfileHevcVldMain = {
    0x5b11d51b,
    0x2f4c,
    0x4452,
    {0xbc, 0xc3, 0x09, 0xf2, 0xa1, 0x16, 0x0c, 0xc0}};

HRESULT EnumerateHevcDecoders(IMFActivate*** activates, UINT32* count) {
  MFT_REGISTER_TYPE_INFO input{MFMediaType_Video, kVideoFormatHevc};
  return MFTEnumEx(
      MFT_CATEGORY_VIDEO_DECODER,
      MFT_ENUM_FLAG_SYNCMFT | MFT_ENUM_FLAG_ASYNCMFT | MFT_ENUM_FLAG_HARDWARE |
          MFT_ENUM_FLAG_SORTANDFILTER,
      &input, nullptr, activates, count);
}

HRESULT CreateHevcVideoDevice(ComPtr<ID3D11Device>& device,
                              ComPtr<ID3D11DeviceContext>& context) {
  ComPtr<IDXGIFactory1> factory;
  HRESULT hr = CreateDXGIFactory1(IID_PPV_ARGS(&factory));
  if (FAILED(hr)) return hr;
  for (UINT index = 0;; ++index) {
    ComPtr<IDXGIAdapter1> adapter;
    hr = factory->EnumAdapters1(index, &adapter);
    if (hr == DXGI_ERROR_NOT_FOUND) break;
    if (FAILED(hr)) return hr;
    DXGI_ADAPTER_DESC1 adapter_desc{};
    if (FAILED(adapter->GetDesc1(&adapter_desc)) ||
        (adapter_desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE)) {
      continue;
    }
    ComPtr<ID3D11Device> candidate;
    ComPtr<ID3D11DeviceContext> candidate_context;
    hr = D3D11CreateDevice(
        adapter.Get(), D3D_DRIVER_TYPE_UNKNOWN, nullptr,
        D3D11_CREATE_DEVICE_VIDEO_SUPPORT | D3D11_CREATE_DEVICE_BGRA_SUPPORT,
        nullptr, 0, D3D11_SDK_VERSION, &candidate, nullptr, &candidate_context);
    if (FAILED(hr)) continue;
    ComPtr<ID3D11VideoDevice> video;
    if (FAILED(candidate.As(&video))) continue;
    BOOL supported = FALSE;
    hr = video->CheckVideoDecoderFormat(&kD3d11DecoderProfileHevcVldMain,
                                        DXGI_FORMAT_NV12, &supported);
    if (FAILED(hr) || !supported) continue;
    D3D11_VIDEO_DECODER_DESC desc{};
    desc.Guid = kD3d11DecoderProfileHevcVldMain;
    desc.SampleWidth = 1920;
    desc.SampleHeight = 1080;
    desc.OutputFormat = DXGI_FORMAT_NV12;
    UINT configs = 0;
    if (FAILED(video->GetVideoDecoderConfigCount(&desc, &configs)) || !configs) {
      continue;
    }
    ComPtr<ID3D10Multithread> multithread;
    if (FAILED(candidate.As(&multithread))) continue;
    multithread->SetMultithreadProtected(TRUE);
    device = candidate;
    context = candidate_context;
    return S_OK;
  }
  return MF_E_UNSUPPORTED_D3D_TYPE;
}

struct FrameOutput {
  DecodedFrame frame;
  NativeVideoFrameRef native;
};

}  // namespace

class D3D11HevcDecoder::Impl {
 public:
  Impl(std::shared_ptr<SystemClock> clock, bool native_output)
      : clock_(std::move(clock)),
        native_output_(native_output),
        pool_(NativeNv12FramePool::Create()) {}

  ~Impl() {
    worker_.Call([this] { Reset(); });
  }

  void Reset() {
    if (decoder_ && using_d3d11_) {
      decoder_->ProcessMessage(MFT_MESSAGE_SET_D3D_MANAGER, 0);
    }
    wmf::ShutdownTransform(decoder_);
    reusable_output_sample_.Reset();
    staging_.Reset();
    manager_.Reset();
    context_.Reset();
    device_.Reset();
    using_d3d11_ = false;
    timestamps_.clear();
    coded_width_ = coded_height_ = 0;
    width_ = height_ = crop_x_ = crop_y_ = 0;
    default_stride_ = 0;
  }

  HRESULT SetOutputType() {
    for (DWORD i = 0;; ++i) {
      ComPtr<IMFMediaType> type;
      HRESULT hr = decoder_->GetOutputAvailableType(0, i, &type);
      if (FAILED(hr)) return hr;
      GUID subtype{};
      if (FAILED(type->GetGUID(MF_MT_SUBTYPE, &subtype)) ||
          subtype != MFVideoFormat_NV12) {
        continue;
      }
      hr = decoder_->SetOutputType(0, type.Get(), 0);
      if (FAILED(hr)) continue;

      UINT32 frame_w = 0, frame_h = 0;
      hr = MFGetAttributeSize(type.Get(), MF_MT_FRAME_SIZE, &frame_w, &frame_h);
      if (FAILED(hr)) return hr;
      coded_width_ = frame_w;
      coded_height_ = frame_h;
      width_ = frame_w;
      height_ = frame_h;
      crop_x_ = crop_y_ = 0;

      UINT32 stride_u32 = 0;
      if (SUCCEEDED(type->GetUINT32(MF_MT_DEFAULT_STRIDE, &stride_u32)) &&
          static_cast<LONG>(stride_u32) > 0) {
        default_stride_ = static_cast<UINT>(static_cast<LONG>(stride_u32));
      } else {
        default_stride_ = frame_w;
      }

      MFVideoArea area{};
      if (SUCCEEDED(type->GetBlob(MF_MT_MINIMUM_DISPLAY_APERTURE,
                                  reinterpret_cast<UINT8*>(&area), sizeof(area),
                                  nullptr)) ||
          SUCCEEDED(type->GetBlob(MF_MT_GEOMETRIC_APERTURE,
                                  reinterpret_cast<UINT8*>(&area), sizeof(area),
                                  nullptr))) {
        if (area.OffsetX.value < 0 || area.OffsetY.value < 0 ||
            area.OffsetX.fract || area.OffsetY.fract || area.Area.cx <= 0 ||
            area.Area.cy <= 0) {
          return MF_E_INVALIDMEDIATYPE;
        }
        crop_x_ = static_cast<UINT>(area.OffsetX.value);
        crop_y_ = static_cast<UINT>(area.OffsetY.value);
        width_ = static_cast<UINT>(area.Area.cx);
        height_ = static_cast<UINT>(area.Area.cy);
      }

      size_t size = 0;
      if (width_ > 8192 || height_ > 8192 || (crop_x_ & 1u) || (crop_y_ & 1u) ||
          !GetNv12FrameSize(width_, height_, &size)) {
        return MF_E_INVALIDMEDIATYPE;
      }
      staging_.Reset();
      reusable_output_sample_.Reset();
      return S_OK;
    }
  }

  HRESULT ConfigureDecoder(IMFActivate* activation, bool try_d3d11) {
    HRESULT hr = activation->ActivateObject(IID_PPV_ARGS(&decoder_));
    if (FAILED(hr)) return hr;

    ComPtr<IMFAttributes> attributes;
    hr = decoder_->GetAttributes(&attributes);
    if (FAILED(hr)) return hr;

    const bool is_async =
        MFGetAttributeUINT32(attributes.Get(), MF_TRANSFORM_ASYNC, FALSE) != 0;
    if (is_async) {
      // Microsoft HEVC Video Extensions exposes a synchronous MFT; reject
      // unexpected async-only MFTs here rather than deadlocking ProcessOutput.
      return MF_E_UNSUPPORTED_FORMAT;
    }

    attributes->SetUINT32(CODECAPI_AVLowLatencyMode, TRUE);

    using_d3d11_ = false;
    if (try_d3d11 &&
        MFGetAttributeUINT32(attributes.Get(), MF_SA_D3D11_AWARE, FALSE) != 0) {
      if (SUCCEEDED(CreateHevcVideoDevice(device_, context_))) {
        UINT token = 0;
        if (SUCCEEDED(MFCreateDXGIDeviceManager(&token, &manager_)) &&
            SUCCEEDED(manager_->ResetDevice(device_.Get(), token)) &&
            SUCCEEDED(decoder_->ProcessMessage(
                MFT_MESSAGE_SET_D3D_MANAGER,
                reinterpret_cast<ULONG_PTR>(manager_.Get())))) {
          using_d3d11_ = true;
        } else {
          manager_.Reset();
          context_.Reset();
          device_.Reset();
        }
      }
    }

    ComPtr<IMFMediaType> input;
    hr = MFCreateMediaType(&input);
    if (FAILED(hr)) return hr;
    input->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    input->SetGUID(MF_MT_SUBTYPE, kVideoFormatHevc);
    // HEVCVideoExtension requires an initial placeholder MF_MT_FRAME_SIZE before
    // SetOutputType(NV12) succeeds; ProcessOutput emits STREAM_CHANGE once the
    // real SPS dimensions and conformance crop aperture are parsed.
    MFSetAttributeSize(input.Get(), MF_MT_FRAME_SIZE, 1920, 1080);
    hr = decoder_->SetInputType(0, input.Get(), 0);
    if (FAILED(hr)) return hr;

    hr = SetOutputType();
    if (FAILED(hr)) return hr;

    hr = decoder_->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, 0);
    if (FAILED(hr)) return hr;
    return decoder_->ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM, 0);
  }

  HRESULT Initialize(bool try_d3d11) {
    Reset();
    if (FAILED(worker_.Status())) return worker_.Status();

    IMFActivate** activations = nullptr;
    UINT32 count = 0;
    HRESULT hr = EnumerateHevcDecoders(&activations, &count);
    if (FAILED(hr)) return hr;

    hr = MF_E_TOPO_CODEC_NOT_FOUND;
    for (UINT32 i = 0; i < count; ++i) {
      if (!decoder_) {
        const HRESULT cfg_hr = ConfigureDecoder(activations[i], try_d3d11);
        if (SUCCEEDED(cfg_hr)) {
          WCHAR name[256]{};
          activations[i]->GetString(MFT_FRIENDLY_NAME_Attribute, name, 256,
                                    nullptr);
          char utf8[1024]{};
          WideCharToMultiByte(CP_UTF8, 0, name, -1, utf8, sizeof(utf8), nullptr,
                              nullptr);
          LOG_INFO("Media Foundation HEVC decoder ({}): {}",
                   using_d3d11_ ? "D3D11VA" : "SW", utf8);
          activations[i]->DetachObject();
          hr = S_OK;
        } else {
          hr = cfg_hr;
          Reset();
          activations[i]->ShutdownObject();
        }
      }
      activations[i]->Release();
    }
    CoTaskMemFree(activations);
    return decoder_ ? S_OK : hr;
  }

  HRESULT PopulateOutputFrame(const uint8_t* source, size_t pitch,
                              UINT surface_height, IMFSample* sample,
                              FrameOutput& output) {
    if (!source || pitch < static_cast<size_t>(crop_x_) + width_ ||
        surface_height < crop_y_ + height_) {
      return MF_E_BUFFERTOOSMALL;
    }
    size_t size = 0;
    if (!GetNv12FrameSize(width_, height_, &size)) {
      return MF_E_INVALIDMEDIATYPE;
    }
    uint8_t* destination = nullptr;
    if (native_output_) {
      auto* native = pool_->Acquire(width_, height_);
      if (!native) return E_OUTOFMEMORY;
      output.native = NativeVideoFrameRef(native->Descriptor());
      destination = native->YPlane();
      native->Release();
      output.frame.SetSize(size);
    } else {
      output.frame = DecodedFrame(size, width_, height_);
      destination = output.frame.MutableBuffer();
    }

    for (UINT y = 0; y < height_; ++y) {
      std::memcpy(destination + static_cast<size_t>(y) * width_,
                  source + static_cast<size_t>(crop_y_ + y) * pitch + crop_x_,
                  width_);
    }
    const size_t pixels = static_cast<size_t>(width_) * height_;
    for (UINT y = 0; y < height_ / 2u; ++y) {
      std::memcpy(
          destination + pixels + static_cast<size_t>(y) * width_,
          source + static_cast<size_t>(surface_height + crop_y_ / 2u + y) *
                       pitch +
              crop_x_,
          width_);
    }

    LONGLONG time = 0;
    int64_t captured_ts = 0;
    int64_t received_ts = 0;
    if (SUCCEEDED(sample->GetSampleTime(&time))) {
      auto metadata = timestamps_.find(time);
      if (metadata != timestamps_.end()) {
        captured_ts = metadata->second.first;
        received_ts = metadata->second.second;
        timestamps_.erase(metadata);
      } else if (!timestamps_.empty()) {
        captured_ts = timestamps_.begin()->second.first;
        received_ts = timestamps_.begin()->second.second;
        timestamps_.erase(timestamps_.begin());
      }
    }
    output.frame.SetWidth(width_);
    output.frame.SetHeight(height_);
    output.frame.SetDecodedWidth(width_);
    output.frame.SetDecodedHeight(height_);
    output.frame.SetCapturedTimestamp(captured_ts);
    output.frame.SetReceivedTimestamp(received_ts);
    output.frame.SetDecodedTimestamp(clock_->CurrentTime());
    return S_OK;
  }

  HRESULT CopyFrame(IMFSample* sample, FrameOutput& output) {
    ComPtr<IMFMediaBuffer> buffer;
    HRESULT hr = sample->GetBufferByIndex(0, &buffer);
    if (FAILED(hr)) return hr;

    ComPtr<IMFDXGIBuffer> dxgi;
    if (using_d3d11_ && SUCCEEDED(buffer.As(&dxgi)) && dxgi) {
      ComPtr<ID3D11Texture2D> texture;
      hr = dxgi->GetResource(IID_PPV_ARGS(&texture));
      if (FAILED(hr)) return hr;
      UINT subresource = 0;
      hr = dxgi->GetSubresourceIndex(&subresource);
      if (FAILED(hr)) return hr;
      D3D11_TEXTURE2D_DESC desc{};
      texture->GetDesc(&desc);
      if (desc.Format != DXGI_FORMAT_NV12 || width_ > desc.Width ||
          height_ > desc.Height || crop_x_ > desc.Width - width_ ||
          crop_y_ > desc.Height - height_) {
        return MF_E_INVALIDMEDIATYPE;
      }
      D3D11_TEXTURE2D_DESC staged{};
      if (staging_) staging_->GetDesc(&staged);
      if (!staging_ || staged.Width != desc.Width ||
          staged.Height != desc.Height) {
        staging_.Reset();
        staged = desc;
        staged.MipLevels = staged.ArraySize = 1;
        staged.Usage = D3D11_USAGE_STAGING;
        staged.BindFlags = staged.MiscFlags = 0;
        staged.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        hr = device_->CreateTexture2D(&staged, nullptr, &staging_);
        if (FAILED(hr)) return hr;
      }
      context_->CopySubresourceRegion(staging_.Get(), 0, 0, 0, 0, texture.Get(),
                                      subresource, nullptr);
      D3D11_MAPPED_SUBRESOURCE mapped{};
      hr = context_->Map(staging_.Get(), 0, D3D11_MAP_READ, 0, &mapped);
      if (FAILED(hr)) return hr;
      hr = PopulateOutputFrame(static_cast<const uint8_t*>(mapped.pData),
                               mapped.RowPitch, desc.Height, sample, output);
      context_->Unmap(staging_.Get(), 0);
      return hr;
    }

    // Software / CPU buffer output fallback path.
    ComPtr<IMF2DBuffer> buffer_2d;
    if (SUCCEEDED(buffer.As(&buffer_2d)) && buffer_2d) {
      BYTE* scanline0 = nullptr;
      LONG pitch = 0;
      hr = buffer_2d->Lock2D(&scanline0, &pitch);
      if (FAILED(hr)) return hr;
      if (pitch <= 0) {
        buffer_2d->Unlock2D();
        return MF_E_INVALIDMEDIATYPE;
      }
      hr = PopulateOutputFrame(scanline0, static_cast<size_t>(pitch),
                               coded_height_, sample, output);
      buffer_2d->Unlock2D();
      return hr;
    }

    BYTE* data = nullptr;
    DWORD cur_len = 0;
    hr = buffer->Lock(&data, nullptr, &cur_len);
    if (FAILED(hr)) return hr;
    const size_t pitch = default_stride_ >= width_ ? default_stride_ : width_;
    const size_t needed = pitch * coded_height_ * 3u / 2u;
    if (cur_len < needed) {
      buffer->Unlock();
      return MF_E_BUFFERTOOSMALL;
    }
    hr = PopulateOutputFrame(data, pitch, coded_height_, sample, output);
    buffer->Unlock();
    return hr;
  }

  HRESULT ReadFrames(std::vector<FrameOutput>& output) {
    for (unsigned i = 0; i < 64; ++i) {
      ComPtr<IMFSample> sample;
      const HRESULT hr = wmf::ReadOutput(decoder_.Get(), 0,
                                         reusable_output_sample_, sample);
      if (hr == MF_E_TRANSFORM_NEED_MORE_INPUT) return S_OK;
      if (hr == MF_E_TRANSFORM_STREAM_CHANGE) {
        const HRESULT changed = SetOutputType();
        if (FAILED(changed)) return changed;
        continue;
      }
      if (FAILED(hr)) return hr;
      if (!sample) continue;
      FrameOutput frame;
      const HRESULT copied = CopyFrame(sample.Get(), frame);
      if (FAILED(copied)) return copied;
      output.push_back(std::move(frame));
    }
    return E_UNEXPECTED;
  }

  HRESULT DecodeFrame(const ReceivedFrame& frame,
                      std::vector<FrameOutput>& output) {
    if (!decoder_) return MF_E_NOT_INITIALIZED;
    ComPtr<IMFSample> sample;
    const LONGLONG time = next_time_;
    next_time_ += 166667;
    HRESULT hr =
        wmf::CreateSample(frame.Buffer(), frame.Size(), time, 166667, sample);
    if (FAILED(hr)) return hr;
    hr = decoder_->ProcessInput(0, sample.Get(), 0);
    if (hr == MF_E_NOTACCEPTING) {
      hr = ReadFrames(output);
      if (FAILED(hr)) return hr;
      hr = decoder_->ProcessInput(0, sample.Get(), 0);
    }
    if (FAILED(hr)) return hr;
    timestamps_[time] = {frame.CapturedTimestamp(), frame.ReceivedTimestamp()};
    if (timestamps_.size() > 64) {
      timestamps_.erase(timestamps_.begin());
    }
    return ReadFrames(output);
  }

  wmf::CodecThread worker_;
  std::shared_ptr<SystemClock> clock_;
  bool native_output_ = false;
  bool initialized_ = false;
  bool using_d3d11_ = false;
  std::shared_ptr<NativeNv12FramePool> pool_;
  ComPtr<ID3D11Device> device_;
  ComPtr<ID3D11DeviceContext> context_;
  ComPtr<IMFDXGIDeviceManager> manager_;
  ComPtr<IMFTransform> decoder_;
  ComPtr<ID3D11Texture2D> staging_;
  wmf::ReusableOutputSample reusable_output_sample_;
  UINT coded_width_ = 0, coded_height_ = 0;
  UINT width_ = 0, height_ = 0, crop_x_ = 0, crop_y_ = 0;
  UINT default_stride_ = 0;
  LONGLONG next_time_ = 0;
  std::map<LONGLONG, std::pair<int64_t, int64_t>> timestamps_;
};

D3D11HevcDecoder::D3D11HevcDecoder(std::shared_ptr<SystemClock> clock,
                                   bool native_video_output)
    : impl_(std::make_unique<Impl>(std::move(clock), native_video_output)) {}

D3D11HevcDecoder::~D3D11HevcDecoder() = default;

bool D3D11HevcDecoder::IsSupported() {
  wmf::CodecThread worker;
  if (FAILED(worker.Status())) return false;
  return worker.Call([] {
    IMFActivate** activations = nullptr;
    UINT32 count = 0;
    const HRESULT hr = EnumerateHevcDecoders(&activations, &count);
    for (UINT32 i = 0; i < count; ++i) activations[i]->Release();
    CoTaskMemFree(activations);
    return SUCCEEDED(hr) && count > 0;
  });
}

int D3D11HevcDecoder::Init() {
  return impl_->worker_.Call([&] {
    HRESULT hr = impl_->Initialize(true);
    if (FAILED(hr)) {
      LOG_WARN(
          "D3D11VA HEVC initialization failed (0x{:08x}); retrying WMF HEVC "
          "software mode",
          static_cast<uint32_t>(hr));
      hr = impl_->Initialize(false);
    }
    impl_->initialized_ = SUCCEEDED(hr);
    if (!impl_->initialized_) {
      LOG_ERROR("WMF HEVC decoder initialization failed (0x{:08x})",
                static_cast<uint32_t>(hr));
    }
    return impl_->initialized_ ? 0 : -1;
  });
}

int D3D11HevcDecoder::Decode(
    std::unique_ptr<ReceivedFrame> frame,
    std::function<void(const DecodedFrame*)> callback) {
  if (!frame || !frame->Buffer() || !frame->Size() ||
      frame->Size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
    return -1;
  }
  std::vector<FrameOutput> output;
  const int result = impl_->worker_.Call([&] {
    if (!impl_->initialized_) return -1;
    const HRESULT hr = impl_->DecodeFrame(*frame, output);
    if (SUCCEEDED(hr)) return 0;
    LOG_WARN("WMF HEVC decoding failed (0x{:08x}); reinitializing decoder",
             static_cast<uint32_t>(hr));
    // Reinitialize while preserving D3D11 first so transient packet corruption
    // does not permanently drop back to software decoding.
    HRESULT reinit_hr = impl_->Initialize(impl_->using_d3d11_);
    if (FAILED(reinit_hr) && impl_->using_d3d11_) {
      LOG_WARN(
          "D3D11VA HEVC recovery failed (0x{:08x}); falling back to software",
          static_cast<uint32_t>(reinit_hr));
      reinit_hr = impl_->Initialize(false);
    }
    if (FAILED(reinit_hr)) {
      impl_->initialized_ = false;
      return -1;
    }
    if (!wmf::IsHevcKeyFrame(frame->Buffer(), frame->Size())) {
      return -1;
    }
    return SUCCEEDED(impl_->DecodeFrame(*frame, output)) ? 0 : -1;
  });
  for (auto& decoded : output) {
    decoded.frame.SetNativeFrame(decoded.native.Get());
    if (callback) callback(&decoded.frame);
  }
  return result;
}

std::string D3D11HevcDecoder::GetDecoderName() const {
  return impl_->worker_.Call([&]() -> std::string {
    return impl_->using_d3d11_ ? "D3D11VA (HEVC)"
                               : "Media Foundation (HEVC, SW)";
  });
}

}  // namespace minirtc
