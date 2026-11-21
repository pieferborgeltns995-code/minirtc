#include "amf_hevc_encoder.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <d3d11.h>
#include <wrl/client.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

#include "components/Component.h"
#include "components/ComponentCaps.h"
#include "components/VideoEncoderHEVC.h"
#include "core/Buffer.h"
#include "core/Context.h"
#include "core/Factory.h"
#include "core/Surface.h"
#include "log.h"
#include "wmf/wmf_hevc_encoder.h"
#include "wmf/wmf_utils.h"

namespace minirtc {
namespace {

using Microsoft::WRL::ComPtr;

constexpr uint8_t kHevcNaluVps = 32;
constexpr uint8_t kHevcNaluSps = 33;
constexpr uint8_t kHevcNaluPps = 34;
constexpr uint8_t kHevcNaluAud = 35;

bool ValidConfig(const MediaCodecConfig& config) {
  size_t size = 0;
  return config.init_width > 0 && config.init_height > 0 &&
         config.init_width <= 8192 && config.init_height <= 8192 &&
         GetNv12FrameSize(config.init_width, config.init_height, &size) &&
         config.init_bitrate > 0 && config.max_bitrate > 0 &&
         config.max_frame_rate > 0 && config.max_frame_rate <= 240 &&
         config.key_frame_interval > 0;
}

const uint8_t* ResolveCpuNv12(const RawFrame& raw,
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

bool CopyNv12ToStridedPlanes(const uint8_t* src_nv12, int width, int height,
                             uint8_t* dst_y, int dst_y_pitch, uint8_t* dst_uv,
                             int dst_uv_pitch) {
  if (!src_nv12 || !dst_y || !dst_uv || width <= 0 || height <= 0 ||
      dst_y_pitch < width || dst_uv_pitch < width || (width & 1) != 0 ||
      (height & 1) != 0) {
    return false;
  }
  if (dst_y_pitch == width && dst_uv_pitch == width &&
      dst_uv == dst_y + static_cast<size_t>(width) * height) {
    std::memcpy(dst_y, src_nv12,
                static_cast<size_t>(width) * height * 3 / 2);
    return true;
  }
  for (int y = 0; y < height; ++y) {
    std::memcpy(dst_y + static_cast<size_t>(y) * dst_y_pitch,
                src_nv12 + static_cast<size_t>(y) * width,
                static_cast<size_t>(width));
  }
  const uint8_t* src_uv = src_nv12 + static_cast<size_t>(width) * height;
  const int uv_h = height / 2;
  for (int y = 0; y < uv_h; ++y) {
    std::memcpy(dst_uv + static_cast<size_t>(y) * dst_uv_pitch,
                src_uv + static_cast<size_t>(y) * width,
                static_cast<size_t>(width));
  }
  return true;
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
    const double in_p50 = Percentile(input_ms_, 0.50);
    const double in_p95 = Percentile(input_ms_, 0.95);
    const double hw_p50 = Percentile(hw_ms_, 0.50);
    const double hw_p95 = Percentile(hw_ms_, 0.95);
    const double avg_kb = static_cast<double>(total_bytes_) /
                          (static_cast<double>(n) * 1024.0);
    LOG_INFO(
        "{} stats ({:.1f}s): {}x{} d3d11={} frames={} fps={:.1f} kf={} "
        "enc_ms[p50={:.2f} p95={:.2f} max={:.2f}] "
        "wait_ms[p50={:.2f} p95={:.2f}] input_ms[p50={:.2f} p95={:.2f}] "
        "hw_ms[p50={:.2f} p95={:.2f}] avg_kb={:.1f}",
        encoder_tag, elapsed_s, encode_width, encode_height, using_d3d11, n,
        static_cast<double>(n) / elapsed_s, keyframes_, enc_p50, enc_p95,
        enc_max, wait_p50, wait_p95, in_p50, in_p95, hw_p50, hw_p95, avg_kb);
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

AMF_RESULT LoadAmfFactory(HMODULE* out_dll, amf::AMFFactory** out_factory) {
  if (!out_dll || !out_factory) return AMF_INVALID_ARG;
  *out_dll = nullptr;
  *out_factory = nullptr;
  HMODULE dll = LoadLibraryW(AMF_DLL_NAME);
  if (!dll) {
    return AMF_FAIL;
  }
  auto query_version = reinterpret_cast<AMFQueryVersion_Fn>(
      GetProcAddress(dll, AMF_QUERY_VERSION_FUNCTION_NAME));
  auto init_fn = reinterpret_cast<AMFInit_Fn>(
      GetProcAddress(dll, AMF_INIT_FUNCTION_NAME));
  if (!init_fn) {
    FreeLibrary(dll);
    return AMF_FAIL;
  }
  if (query_version) {
    amf_uint64 driver_version = 0;
    query_version(&driver_version);
  }
  amf::AMFFactory* factory = nullptr;
  const AMF_RESULT res = init_fn(AMF_FULL_VERSION, &factory);
  if (res != AMF_OK || !factory) {
    FreeLibrary(dll);
    return res != AMF_OK ? res : AMF_FAIL;
  }
  *out_dll = dll;
  *out_factory = factory;
  return AMF_OK;
}

template <typename T>
void SetOptionalProp(amf::AMFPropertyStorage* storage, const wchar_t* name,
                     const char* label, const T& value) {
  if (!storage) return;
  const AMF_RESULT res = storage->SetProperty(name, value);
  if (res != AMF_OK) {
    LOG_WARN("AMD AMF HEVC property {} failed: res={}", label,
             static_cast<int>(res));
  }
}

int64_t ReadPropInt64(amf::AMFPropertyStorage* storage, const wchar_t* name) {
  if (!storage) return -1;
  amf::AMFVariant var;
  if (storage->GetProperty(name, &var) == AMF_OK &&
      var.type == amf::AMF_VARIANT_INT64) {
    return static_cast<int64_t>(var.int64Value);
  }
  return -1;
}

int ReadPropBool(amf::AMFPropertyStorage* storage, const wchar_t* name) {
  if (!storage) return -1;
  amf::AMFVariant var;
  if (storage->GetProperty(name, &var) == AMF_OK) {
    if (var.type == amf::AMF_VARIANT_BOOL) {
      return var.boolValue ? 1 : 0;
    }
    if (var.type == amf::AMF_VARIANT_INT64) {
      return var.int64Value != 0 ? 1 : 0;
    }
  }
  return -1;
}

bool IsHardwareAccelerated(amf::AMFComponent* encoder) {
  if (!encoder) return false;
  amf::AMFCapsPtr caps;
  if (encoder->GetCaps(&caps) != AMF_OK || !caps) {
    return true;
  }
  const amf::AMF_ACCELERATION_TYPE accel = caps->GetAccelerationType();
  return accel == amf::AMF_ACCEL_HARDWARE || accel == amf::AMF_ACCEL_GPU;
}

bool ProbeAmfHevcSupport() {
  HMODULE dll = nullptr;
  amf::AMFFactory* factory = nullptr;
  if (LoadAmfFactory(&dll, &factory) != AMF_OK) {
    return false;
  }
  bool supported = false;
  {
    amf::AMFContextPtr context;
    amf::AMFComponentPtr encoder;
    if (factory->CreateContext(&context) == AMF_OK && context &&
        context->InitDX11(nullptr, amf::AMF_DX11_0) == AMF_OK &&
        factory->CreateComponent(context, AMFVideoEncoder_HEVC, &encoder) ==
            AMF_OK &&
        encoder && IsHardwareAccelerated(encoder)) {
      supported = true;
    }
    if (encoder) {
      encoder->Terminate();
      encoder = nullptr;
    }
    if (context) {
      context->Terminate();
      context = nullptr;
    }
  }
  factory = nullptr;
  FreeLibrary(dll);
  return supported;
}

void StoreParameterSetNalu(const uint8_t* payload, size_t size, uint8_t type,
                           std::vector<uint8_t>& vps, std::vector<uint8_t>& sps,
                           std::vector<uint8_t>& pps) {
  static constexpr uint8_t kStartCode[4] = {0, 0, 0, 1};
  std::vector<uint8_t>* target = nullptr;
  if (type == kHevcNaluVps) {
    target = &vps;
  } else if (type == kHevcNaluSps) {
    target = &sps;
  } else if (type == kHevcNaluPps) {
    target = &pps;
  }
  if (target && size > 0) {
    target->clear();
    target->reserve(4 + size);
    target->insert(target->end(), kStartCode, kStartCode + 4);
    target->insert(target->end(), payload, payload + size);
  }
}

void CacheHevcParameterSetsFromBytes(const uint8_t* data, size_t size,
                                     std::vector<uint8_t>& vps,
                                     std::vector<uint8_t>& sps,
                                     std::vector<uint8_t>& pps) {
  if (!data || size == 0) return;
  const auto nalus = wmf::AnnexBHevcNalus(data, size);
  bool found = false;
  for (const auto& nalu : nalus) {
    if (nalu.type == kHevcNaluVps || nalu.type == kHevcNaluSps ||
        nalu.type == kHevcNaluPps) {
      StoreParameterSetNalu(data + nalu.offset, nalu.size, nalu.type, vps, sps,
                            pps);
      found = true;
    }
  }
  if (found) return;

  // Fallback for ISO/IEC 14496-15 HEVCDecoderConfigurationRecord (hvcC).
  if (size >= 23 && data[0] == 1) {
    const uint8_t num_arrays = data[22];
    size_t pos = 23;
    for (uint8_t a = 0; a < num_arrays && pos + 3 <= size; ++a) {
      const uint8_t nalu_type = data[pos] & 0x3Fu;
      const uint16_t num_nalus =
          (static_cast<uint16_t>(data[pos + 1]) << 8) | data[pos + 2];
      pos += 3;
      for (uint16_t n = 0; n < num_nalus && pos + 2 <= size; ++n) {
        const uint16_t nalu_len =
            (static_cast<uint16_t>(data[pos]) << 8) | data[pos + 1];
        pos += 2;
        if (pos + nalu_len > size) return;
        if (nalu_len >= 2) {
          const uint8_t parsed_type =
              static_cast<uint8_t>((data[pos] >> 1) & 0x3Fu);
          StoreParameterSetNalu(data + pos, nalu_len,
                                parsed_type ? parsed_type : nalu_type, vps, sps,
                                pps);
        }
        pos += nalu_len;
      }
    }
  }
}

}  // namespace

class AmfHevcEncoder::Impl {
 public:
  explicit Impl(std::shared_ptr<SystemClock> clock)
      : clock_(std::move(clock)) {}

  ~Impl() {
    callback_worker_.Flush();
    worker_.Call([this] {
      Reset(true);
      fallback_.reset();
    });
  }

  void Reset(bool unload_dll = false) {
    if (encoder_) {
      encoder_->Terminate();
      encoder_ = nullptr;
    }
    if (context_) {
      context_->Terminate();
      context_ = nullptr;
    }
    if (unload_dll) {
      factory_ = nullptr;
      if (amf_dll_) {
        FreeLibrary(amf_dll_);
        amf_dll_ = nullptr;
      }
    }
    d3d_crop_tex_.Reset();
    d3d_crop_w_ = 0;
    d3d_crop_h_ = 0;
    d3d_context_.Reset();
    d3d_device_.Reset();
    d3d_tex_width_ = 0;
    d3d_tex_height_ = 0;
    using_d3d11_ = false;
    vps_.clear();
    sps_.clear();
    pps_.clear();
    sequence_ = 0;
    force_idr_ = true;
    stats_.Reset();
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

  void ApplyRateControlProperties(bool initial_config) {
    if (!encoder_) return;
    const amf_int64 fps = (std::max)(1, config_.max_frame_rate);
    const amf_int64 target_bps = (std::max)(10000, config_.init_bitrate);
    // Align with Sunshine hevc_amf: Peak == Target under vbr_latency and keep
    // VBV sized to 1 frame (floor 200 Kbit) with HRD enforcement disabled.
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_TARGET_BITRATE,
                    "TARGET_BITRATE", target_bps);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_PEAK_BITRATE,
                    "PEAK_BITRATE", target_bps);

    if (!initial_config) {
      return;
    }

    const amf_int64 vbv_bits =
        (std::max)(target_bps / fps, static_cast<amf_int64>(200'000LL));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_VBV_BUFFER_SIZE,
                    "VBV_BUFFER_SIZE", vbv_bits);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_ENFORCE_HRD,
                    "ENFORCE_HRD", false);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_FILLER_DATA_ENABLE,
                    "FILLER_DATA_ENABLE", false);
    SetOptionalProp(encoder_,
                    AMF_VIDEO_ENCODER_HEVC_RATE_CONTROL_SKIP_FRAME_ENABLE,
                    "RATE_CONTROL_SKIP_FRAME_ENABLE", false);
  }

  void LogAppliedProperties(int coded_w, int coded_h) {
    if (!encoder_) return;
    LOG_INFO(
        "AMF HEVC readback ({}x{}): usage={} rc={} lowlatency={} preset={} "
        "level={} target={} peak={} vbv={} hrd={} filler={} skip={} vbaq={}",
        coded_w, coded_h,
        ReadPropInt64(encoder_, AMF_VIDEO_ENCODER_HEVC_USAGE),
        ReadPropInt64(encoder_, AMF_VIDEO_ENCODER_HEVC_RATE_CONTROL_METHOD),
        ReadPropBool(encoder_, AMF_VIDEO_ENCODER_HEVC_LOWLATENCY_MODE),
        ReadPropInt64(encoder_, AMF_VIDEO_ENCODER_HEVC_QUALITY_PRESET),
        ReadPropInt64(encoder_, AMF_VIDEO_ENCODER_HEVC_PROFILE_LEVEL),
        ReadPropInt64(encoder_, AMF_VIDEO_ENCODER_HEVC_TARGET_BITRATE),
        ReadPropInt64(encoder_, AMF_VIDEO_ENCODER_HEVC_PEAK_BITRATE),
        ReadPropInt64(encoder_, AMF_VIDEO_ENCODER_HEVC_VBV_BUFFER_SIZE),
        ReadPropBool(encoder_, AMF_VIDEO_ENCODER_HEVC_ENFORCE_HRD),
        ReadPropBool(encoder_, AMF_VIDEO_ENCODER_HEVC_FILLER_DATA_ENABLE),
        ReadPropBool(encoder_,
                     AMF_VIDEO_ENCODER_HEVC_RATE_CONTROL_SKIP_FRAME_ENABLE),
        ReadPropBool(encoder_, AMF_VIDEO_ENCODER_HEVC_ENABLE_VBAQ));
  }

  AMF_RESULT Configure(int width, int height, ID3D11Device* d3d_device) {
    if (!amf_dll_ || !factory_) {
      AMF_RESULT load_res = LoadAmfFactory(&amf_dll_, &factory_);
      if (load_res != AMF_OK) return load_res;
    }

    AMF_RESULT res = factory_->CreateContext(&context_);
    if (res != AMF_OK || !context_) return res != AMF_OK ? res : AMF_FAIL;

    res = context_->InitDX11(d3d_device, amf::AMF_DX11_0);
    if (res != AMF_OK) return res;
    using_d3d11_ = (d3d_device != nullptr);

    res = factory_->CreateComponent(context_, AMFVideoEncoder_HEVC, &encoder_);
    if (res != AMF_OK || !encoder_) return res != AMF_OK ? res : AMF_FAIL;

    if (!IsHardwareAccelerated(encoder_)) {
      return AMF_NOT_SUPPORTED;
    }

    const amf_uint32 fps =
        static_cast<amf_uint32>((std::max)(1, config_.max_frame_rate));

    res = encoder_->SetProperty(
        AMF_VIDEO_ENCODER_HEVC_USAGE,
        static_cast<amf_int64>(AMF_VIDEO_ENCODER_HEVC_USAGE_ULTRA_LOW_LATENCY));
    if (res != AMF_OK) {
      LOG_WARN("AMD AMF HEVC ULTRA_LOW_LATENCY failed (res={}), using "
               "LOW_LATENCY",
               static_cast<int>(res));
      SetOptionalProp(
          encoder_, AMF_VIDEO_ENCODER_HEVC_USAGE, "USAGE",
          static_cast<amf_int64>(AMF_VIDEO_ENCODER_HEVC_USAGE_LOW_LATENCY));
    }

    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_LOWLATENCY_MODE,
                    "LOWLATENCY_MODE", true);
    SetOptionalProp(
        encoder_, AMF_VIDEO_ENCODER_HEVC_QUALITY_PRESET, "QUALITY_PRESET",
        static_cast<amf_int64>(AMF_VIDEO_ENCODER_HEVC_QUALITY_PRESET_SPEED));
    SetOptionalProp(
        encoder_, AMF_VIDEO_ENCODER_HEVC_PROFILE, "PROFILE",
        static_cast<amf_int64>(AMF_VIDEO_ENCODER_HEVC_PROFILE_MAIN));
    SetOptionalProp(
        encoder_, AMF_VIDEO_ENCODER_HEVC_TIER, "TIER",
        static_cast<amf_int64>(AMF_VIDEO_ENCODER_HEVC_TIER_MAIN));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_COLOR_BIT_DEPTH,
                    "COLOR_BIT_DEPTH",
                    static_cast<amf_int64>(AMF_COLOR_BIT_DEPTH_8));

    // Match Sunshine hevc_amf level selection for 4K and below.
    const int64_t pixels = static_cast<int64_t>(width) * height;
    if (pixels <= 8'912'896) {
      if (pixels * fps <= 534'773'760) {
        SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_PROFILE_LEVEL,
                        "PROFILE_LEVEL",
                        static_cast<amf_int64>(AMF_LEVEL_5_1));
      } else if (pixels * fps <= 1'069'547'520) {
        SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_PROFILE_LEVEL,
                        "PROFILE_LEVEL",
                        static_cast<amf_int64>(AMF_LEVEL_5_2));
      }
    }

    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_FRAMESIZE, "FRAMESIZE",
                    AMFConstructSize(width, height));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_FRAMERATE, "FRAMERATE",
                    AMFConstructRate(fps, 1));
    SetOptionalProp(
        encoder_, AMF_VIDEO_ENCODER_HEVC_RATE_CONTROL_METHOD,
        "RATE_CONTROL_METHOD",
        static_cast<amf_int64>(
            AMF_VIDEO_ENCODER_HEVC_RATE_CONTROL_METHOD_LATENCY_CONSTRAINED_VBR));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_PRE_ANALYSIS_ENABLE,
                    "PRE_ANALYSIS_ENABLE", false);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_PREENCODE_ENABLE,
                    "PREENCODE_ENABLE", false);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_ENABLE_VBAQ,
                    "ENABLE_VBAQ", false);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_MAX_NUM_REFRAMES,
                    "MAX_NUM_REFRAMES", static_cast<amf_int64>(1));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_GOP_SIZE, "GOP_SIZE",
                    static_cast<amf_int64>(config_.key_frame_interval));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_NUM_GOPS_PER_IDR,
                    "NUM_GOPS_PER_IDR", static_cast<amf_int64>(1));
    SetOptionalProp(
        encoder_, AMF_VIDEO_ENCODER_HEVC_HEADER_INSERTION_MODE,
        "HEADER_INSERTION_MODE",
        static_cast<amf_int64>(
            AMF_VIDEO_ENCODER_HEVC_HEADER_INSERTION_MODE_IDR_ALIGNED));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_INPUT_QUEUE_SIZE,
                    "INPUT_QUEUE_SIZE", static_cast<amf_int64>(4));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_HEVC_QUERY_TIMEOUT,
                    "QUERY_TIMEOUT", static_cast<amf_int64>(50));

    ApplyRateControlProperties(true);

    res = encoder_->Init(amf::AMF_SURFACE_NV12, width, height);
    if (res != AMF_OK) return res;

    // FFmpeg amfenc_hevc applies skip_frame after Init() as a dynamic property.
    SetOptionalProp(encoder_,
                    AMF_VIDEO_ENCODER_HEVC_RATE_CONTROL_SKIP_FRAME_ENABLE,
                    "RATE_CONTROL_SKIP_FRAME_ENABLE", false);

    CacheParameterSetsFromExtraData();
    LogAppliedProperties(width, height);
    return AMF_OK;
  }

  void CacheParameterSetsFromExtraData() {
    if (!encoder_) return;
    amf::AMFVariant extra_var;
    if (encoder_->GetProperty(AMF_VIDEO_ENCODER_HEVC_EXTRADATA, &extra_var) ==
            AMF_OK &&
        extra_var.type == amf::AMF_VARIANT_INTERFACE &&
        extra_var.pInterface != nullptr) {
      amf::AMFBufferPtr extra_buf(extra_var.pInterface);
      if (extra_buf && extra_buf->GetNative() && extra_buf->GetSize() > 0) {
        CacheHevcParameterSetsFromBytes(
            static_cast<const uint8_t*>(extra_buf->GetNative()),
            extra_buf->GetSize(), vps_, sps_, pps_);
      }
    }
  }

  AMF_RESULT Initialize(ID3D11Device* d3d_device, int d3d_tex_w,
                        int d3d_tex_h) {
    Reset();
    if (FAILED(worker_.Status())) return AMF_FAIL;

    AMF_RESULT last_res = AMF_FAIL;
    if (d3d_device) {
      last_res = Configure(source_width_, source_height_, d3d_device);
      if (last_res == AMF_OK) {
        d3d_device_ = d3d_device;
        d3d_tex_width_ = d3d_tex_w;
        d3d_tex_height_ = d3d_tex_h;
        d3d11_failed_device_ = nullptr;
      } else {
        LOG_WARN(
            "AMD AMF HEVC D3D11 config {}x{} failed (res={}); falling back to "
            "CPU surface input",
            source_width_, source_height_, static_cast<int>(last_res));
        d3d11_failed_device_ = d3d_device;
        Reset();
      }
    }

    if (!encoder_) {
      last_res = Configure(source_width_, source_height_, nullptr);
      if (last_res != AMF_OK) {
        Reset();
      }
    }

    if (!encoder_) {
      LOG_WARN("AMD AMF HEVC encoder initialization failed: res={}",
               static_cast<int>(last_res));
      return last_res;
    }

    LOG_INFO(
        "AMD AMF hardware HEVC encoder initialized ({}): {}x{} target={}bps "
        "fps={} vbr_latency",
        using_d3d11_ ? "D3D11 zero-copy" : "CPU surface", source_width_,
        source_height_, config_.init_bitrate, config_.max_frame_rate);
    reconfigure_ = false;
    return AMF_OK;
  }

  AMF_RESULT PrepareInputSurface(const RawFrame& raw, bool is_d3d11_input,
                                 bool* out_tex_larger_than_visible,
                                 amf::AMFSurfacePtr& out_surface) {
    if (out_tex_larger_than_visible) *out_tex_larger_than_visible = false;
    if (!context_) return AMF_NOT_INITIALIZED;
    AMF_RESULT res = AMF_OK;
    if (is_d3d11_input && using_d3d11_) {
      const MiniRtcNativeVideoFrame* native = raw.NativeFrame();
      auto* tex =
          static_cast<ID3D11Texture2D*>(native->payload.d3d11_nv12.texture);
      const uint32_t subresource = native->payload.d3d11_nv12.subresource_index;
      const bool tex_larger =
          static_cast<int>(native->payload.d3d11_nv12.texture_width) !=
              source_width_ ||
          static_cast<int>(native->payload.d3d11_nv12.texture_height) !=
              source_height_;
      if (out_tex_larger_than_visible) {
        *out_tex_larger_than_visible = tex_larger;
      }

      if (subresource != 0 || (tex_larger && need_d3d11_crop_copy_)) {
        ID3D11Texture2D* exact_tex = nullptr;
        if (FAILED(CopyToExactSizeTexture(tex, subresource, source_width_,
                                          source_height_, &exact_tex)) ||
            !exact_tex) {
          d3d11_failed_device_ = d3d_device_.Get();
          return AMF_FAIL;
        }
        tex = exact_tex;
      }

      res = context_->CreateSurfaceFromDX11Native(tex, &out_surface, nullptr);
      if (res != AMF_OK || !out_surface) {
        if (tex_larger && !need_d3d11_crop_copy_) {
          LOG_WARN(
              "AMD AMF HEVC CreateSurfaceFromDX11Native rejected padded "
              "texture (res={}); switching to GPU exact-size crop copy",
              static_cast<int>(res));
          need_d3d11_crop_copy_ = true;
          ID3D11Texture2D* exact_tex = nullptr;
          if (SUCCEEDED(CopyToExactSizeTexture(
                  static_cast<ID3D11Texture2D*>(
                      native->payload.d3d11_nv12.texture),
                  subresource, source_width_, source_height_, &exact_tex)) &&
              exact_tex) {
            res = context_->CreateSurfaceFromDX11Native(exact_tex, &out_surface,
                                                        nullptr);
          }
        }
        if (res != AMF_OK || !out_surface) {
          d3d11_failed_device_ = d3d_device_.Get();
          return res != AMF_OK ? res : AMF_FAIL;
        }
      } else if (tex_larger && !need_d3d11_crop_copy_) {
        // Input D3D11 capture surfaces may be padded to 64x16; tell AMF the
        // visible crop rectangle (same as FFmpeg amfenc.c).
        const AMF_RESULT crop_res =
            out_surface->SetCrop(0, 0, source_width_, source_height_);
        if (crop_res != AMF_OK) {
          LOG_WARN(
              "AMD AMF HEVC SetCrop(0,0,{},{}) failed (res={}); switching to "
              "GPU exact-size crop copy",
              source_width_, source_height_, static_cast<int>(crop_res));
          need_d3d11_crop_copy_ = true;
          out_surface = nullptr;
          ID3D11Texture2D* exact_tex = nullptr;
          if (FAILED(CopyToExactSizeTexture(tex, subresource, source_width_,
                                            source_height_, &exact_tex)) ||
              !exact_tex) {
            d3d11_failed_device_ = d3d_device_.Get();
            return AMF_FAIL;
          }
          res = context_->CreateSurfaceFromDX11Native(exact_tex, &out_surface,
                                                      nullptr);
          if (res != AMF_OK || !out_surface) {
            d3d11_failed_device_ = d3d_device_.Get();
            return res != AMF_OK ? res : AMF_FAIL;
          }
        }
      }
    } else {
      const uint8_t* src_nv12 = ResolveCpuNv12(raw, cpu_materialize_buf_);
      if (!src_nv12) return AMF_INVALID_ARG;
      res = context_->AllocSurface(amf::AMF_MEMORY_HOST, amf::AMF_SURFACE_NV12,
                                   source_width_, source_height_, &out_surface);
      if (res != AMF_OK || !out_surface) {
        return res != AMF_OK ? res : AMF_FAIL;
      }
      amf::AMFPlane* plane_y = out_surface->GetPlaneAt(0);
      amf::AMFPlane* plane_uv = out_surface->GetPlaneAt(1);
      if (!plane_y || !plane_uv || !plane_y->GetNative() ||
          !plane_uv->GetNative()) {
        return AMF_FAIL;
      }
      if (!CopyNv12ToStridedPlanes(
              src_nv12, source_width_, source_height_,
              static_cast<uint8_t*>(plane_y->GetNative()), plane_y->GetHPitch(),
              static_cast<uint8_t*>(plane_uv->GetNative()),
              plane_uv->GetHPitch())) {
        return AMF_INVALID_ARG;
      }
    }
    return AMF_OK;
  }

  AMF_RESULT ReadOutputBuffer(amf::AMFBuffer* out_buf, const RawFrame& raw,
                              std::vector<EncodedFrame>& frames) {
    if (!out_buf || !out_buf->GetNative() || out_buf->GetSize() == 0) {
      return AMF_FAIL;
    }
    const uint8_t* src = static_cast<const uint8_t*>(out_buf->GetNative());
    output_bytes_.assign(src, src + out_buf->GetSize());
    auto& bytes = output_bytes_;

    const auto nalus = wmf::AnnexBHevcNalus(bytes.data(), bytes.size());
    if (nalus.empty()) {
      LOG_WARN("AMF HEVC output is not a valid Annex-B stream (size={})",
               bytes.size());
      return AMF_FAIL;
    }

    CacheHevcParameterSetsFromBytes(bytes.data(), bytes.size(), vps_, sps_,
                                    pps_);

    bool has_vps = false;
    bool has_sps = false;
    bool has_pps = false;
    bool nalu_is_key = false;
    for (const auto& nalu : nalus) {
      if (nalu.type == kHevcNaluVps) {
        has_vps = true;
      } else if (nalu.type == kHevcNaluSps) {
        has_sps = true;
      } else if (nalu.type == kHevcNaluPps) {
        has_pps = true;
      } else if (nalu.type == 19 || nalu.type == 20 || nalu.type == 21) {
        nalu_is_key = true;
      }
    }

    bool hint_key = false;
    amf::AMFVariant dt_var;
    if (out_buf->GetProperty(AMF_VIDEO_ENCODER_HEVC_OUTPUT_DATA_TYPE,
                             &dt_var) == AMF_OK &&
        dt_var.type == amf::AMF_VARIANT_INT64) {
      hint_key =
          (dt_var.int64Value == AMF_VIDEO_ENCODER_HEVC_OUTPUT_DATA_TYPE_IDR);
    }
    const bool key = hint_key || nalu_is_key;

    if (key && (!has_vps || !has_sps || !has_pps)) {
      if (vps_.empty() || sps_.empty() || pps_.empty()) {
        CacheParameterSetsFromExtraData();
      }
      if (!vps_.empty() && !sps_.empty() && !pps_.empty()) {
        std::vector<uint8_t> headers;
        headers.reserve(vps_.size() + sps_.size() + pps_.size());
        headers.insert(headers.end(), vps_.begin(), vps_.end());
        headers.insert(headers.end(), sps_.begin(), sps_.end());
        headers.insert(headers.end(), pps_.begin(), pps_.end());
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
    return AMF_OK;
  }

  AMF_RESULT EncodeHardware(const RawFrame& raw,
                            std::vector<EncodedFrame>& frames) {
    const auto encode_start = std::chrono::steady_clock::now();
    auto encode_submitted = encode_start;
    double submit_ms = 0.0;
    double wait_poll_ms = 0.0;

    const MiniRtcNativeVideoFrame* native = raw.NativeFrame();
    const bool is_d3d11_input =
        native && native->type == MiniRtcNativeVideoFrameD3d11Nv12 &&
        native->payload.d3d11_nv12.texture != nullptr &&
        static_cast<ID3D11Device*>(native->payload.d3d11_nv12.d3d11_device) !=
            d3d11_failed_device_;

    if (is_d3d11_input) {
      auto* dev =
          static_cast<ID3D11Device*>(native->payload.d3d11_nv12.d3d11_device);
      const int tex_w =
          static_cast<int>(native->payload.d3d11_nv12.texture_width);
      const int tex_h =
          static_cast<int>(native->payload.d3d11_nv12.texture_height);
      const bool size_changed =
          static_cast<int>(raw.Width()) != source_width_ ||
          static_cast<int>(raw.Height()) != source_height_;
      const bool switched_d3d11 = !using_d3d11_;
      const bool dev_changed = dev != d3d_device_.Get();
      const bool tex_changed =
          tex_w != d3d_tex_width_ || tex_h != d3d_tex_height_;
      if (!encoder_ || size_changed || switched_d3d11 || dev_changed ||
          tex_changed || reconfigure_) {
        if (sequence_ > 0) {
          LOG_INFO(
              "Reinitializing AMD AMF HEVC encoder (D3D11): size_chg={} "
              "to_d3d11={} dev_chg={} tex_chg={} reconfig={} {}x{}->{}x{}",
              size_changed, switched_d3d11, dev_changed, tex_changed,
              reconfigure_, source_width_, source_height_, raw.Width(),
              raw.Height());
        }
        source_width_ = static_cast<int>(raw.Width());
        source_height_ = static_cast<int>(raw.Height());
        config_.init_width = source_width_;
        config_.init_height = source_height_;
        if (size_changed || dev_changed) {
          need_d3d11_crop_copy_ = false;
        }
        const AMF_RESULT res = Initialize(dev, tex_w, tex_h);
        if (res != AMF_OK) return res;
      }
    } else {
      const bool size_changed =
          static_cast<int>(raw.Width()) != source_width_ ||
          static_cast<int>(raw.Height()) != source_height_;
      const bool switched_cpu = using_d3d11_;
      if (!encoder_ || size_changed || switched_cpu || reconfigure_) {
        if (sequence_ > 0) {
          LOG_INFO(
              "Reinitializing AMD AMF HEVC encoder (CPU): size_chg={} "
              "to_cpu={} reconfig={} {}x{}->{}x{}",
              size_changed, switched_cpu, reconfigure_, source_width_,
              source_height_, raw.Width(), raw.Height());
        }
        source_width_ = static_cast<int>(raw.Width());
        source_height_ = static_cast<int>(raw.Height());
        config_.init_width = source_width_;
        config_.init_height = source_height_;
        const AMF_RESULT res = Initialize(nullptr, 0, 0);
        if (res != AMF_OK) return res;
      }
    }

    if (!encoder_) return AMF_NOT_INITIALIZED;

    bool tex_larger_than_visible = false;
    amf::AMFSurfacePtr surface;
    AMF_RESULT res = PrepareInputSurface(
        raw, is_d3d11_input, &tex_larger_than_visible, surface);
    if (res != AMF_OK) return res;

    const amf_pts duration = 10'000'000LL / config_.max_frame_rate;
    surface->SetPts(next_time_);
    surface->SetDuration(duration);
    next_time_ += duration;

    if (sequence_ % static_cast<uint64_t>(config_.key_frame_interval) == 0) {
      force_idr_ = true;
    }
    if (force_idr_) {
      SetOptionalProp(
          surface, AMF_VIDEO_ENCODER_HEVC_FORCE_PICTURE_TYPE,
          "FORCE_PICTURE_TYPE",
          static_cast<amf_int64>(AMF_VIDEO_ENCODER_HEVC_PICTURE_TYPE_IDR));
      if (surface->SetProperty(AMF_VIDEO_ENCODER_HEVC_INSERT_HEADER, true) !=
          AMF_OK) {
        SetOptionalProp(surface, AMF_VIDEO_ENCODER_HEVC_INSERT_HEADER,
                        "INSERT_HEADER", static_cast<amf_int64>(1));
      }
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    bool submitted = false;
    while (std::chrono::steady_clock::now() < deadline) {
      if (!submitted) {
        const auto sub_start = std::chrono::steady_clock::now();
        res = encoder_->SubmitInput(surface);
        const auto sub_end = std::chrono::steady_clock::now();
        submit_ms +=
            std::chrono::duration<double, std::milli>(sub_end - sub_start)
                .count();
        if (res == AMF_OK || res == AMF_NEED_MORE_INPUT) {
          submitted = true;
          encode_submitted = sub_end;
          surface = nullptr;
        } else if (res != AMF_INPUT_FULL) {
          if (is_d3d11_input && using_d3d11_ && tex_larger_than_visible &&
              !need_d3d11_crop_copy_) {
            LOG_WARN(
                "AMD AMF HEVC SubmitInput rejected padded D3D11 texture "
                "(res={}); switching to GPU exact-size crop copy",
                static_cast<int>(res));
            need_d3d11_crop_copy_ = true;
          }
          return res;
        }
      }

      amf::AMFDataPtr out_data;
      const auto q_start = std::chrono::steady_clock::now();
      res = encoder_->QueryOutput(&out_data);
      const auto q_end = std::chrono::steady_clock::now();
      const double q_ms =
          std::chrono::duration<double, std::milli>(q_end - q_start).count();
      if (!submitted) {
        wait_poll_ms += q_ms;
      }

      if (res == AMF_OK && out_data) {
        amf::AMFBufferPtr out_buf(out_data);
        res = ReadOutputBuffer(out_buf, raw, frames);
        if (res != AMF_OK) return res;
        ++sequence_;
        if (submitted) {
          const auto encode_end = std::chrono::steady_clock::now();
          const double total_ms =
              std::chrono::duration<double, std::milli>(encode_end -
                                                        encode_start)
                  .count();
          const double hw_ms =
              std::chrono::duration<double, std::milli>(encode_end -
                                                        encode_submitted)
                  .count();
          const size_t encoded_bytes =
              frames.empty() ? 0 : frames.back().Size();
          const bool is_key =
              !frames.empty() &&
              frames.back().FrameType() == VideoFrameType::kVideoFrameKey;
          stats_.Record("AMF HEVC", encode_end, source_width_, source_height_,
                        using_d3d11_, total_ms, wait_poll_ms, submit_ms, hw_ms,
                        encoded_bytes, is_key);
          return AMF_OK;
        }
      }
      if (res != AMF_REPEAT && res != AMF_OK) {
        return res;
      }
      if (!submitted || q_ms < 0.5) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    return AMF_FAIL;
  }

  wmf::CodecThread worker_;
  wmf::CallbackThread callback_worker_;
  std::shared_ptr<SystemClock> clock_;
  MediaCodecConfig config_;
  HMODULE amf_dll_ = nullptr;
  amf::AMFFactory* factory_ = nullptr;
  amf::AMFContextPtr context_;
  amf::AMFComponentPtr encoder_;
  ComPtr<ID3D11Device> d3d_device_;
  ComPtr<ID3D11DeviceContext> d3d_context_;
  ComPtr<ID3D11Texture2D> d3d_crop_tex_;
  int d3d_crop_w_ = 0;
  int d3d_crop_h_ = 0;
  bool need_d3d11_crop_copy_ = false;
  ID3D11Device* d3d11_failed_device_ = nullptr;
  bool using_d3d11_ = false;
  int d3d_tex_width_ = 0;
  int d3d_tex_height_ = 0;
  bool force_idr_ = true;
  bool reconfigure_ = false;
  bool initialized_ = false;
  int source_width_ = 0;
  int source_height_ = 0;
  uint64_t sequence_ = 0;
  amf_pts next_time_ = 0;
  std::vector<uint8_t> vps_;
  std::vector<uint8_t> sps_;
  std::vector<uint8_t> pps_;
  std::vector<uint8_t> output_bytes_;
  std::vector<uint8_t> cpu_materialize_buf_;
  std::unique_ptr<MediaCodec> fallback_;
  EncodeStatsTracker stats_;
};

AmfHevcEncoder::AmfHevcEncoder(std::shared_ptr<SystemClock> clock)
    : impl_(std::make_unique<Impl>(std::move(clock))) {}

AmfHevcEncoder::~AmfHevcEncoder() = default;

bool AmfHevcEncoder::IsSupported() {
  static const bool supported = ProbeAmfHevcSupport();
  return supported;
}

int AmfHevcEncoder::Init(const MediaCodecConfig& config) {
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
    impl_->d3d11_failed_device_ = nullptr;
    impl_->need_d3d11_crop_copy_ = false;
    impl_->initialized_ =
        (impl_->Initialize(nullptr, 0, 0) == AMF_OK);
    return impl_->initialized_ ? 0 : -1;
  });
}

bool AmfHevcEncoder::SupportsNativeFrameInput(
    MiniRtcNativeVideoFrameType frame_type) const {
  return frame_type == MiniRtcNativeVideoFrameD3d11Nv12 ||
         frame_type == MiniRtcNativeVideoFrameCpuNv12;
}

int AmfHevcEncoder::Encode(const RawFrame& raw,
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
    if (!impl_->fallback_) {
      AMF_RESULT res = impl_->EncodeHardware(raw, frames);
      if (res == AMF_OK) return 0;
      if (impl_->using_d3d11_) {
        if (impl_->need_d3d11_crop_copy_) {
          frames.clear();
          res = impl_->EncodeHardware(raw, frames);
          if (res == AMF_OK) return 0;
        }
        LOG_WARN(
            "AMD AMF HEVC D3D11 encoding failed (res={}); retrying with AMF "
            "CPU surface input",
            static_cast<int>(res));
        impl_->d3d11_failed_device_ = impl_->d3d_device_.Get();
        impl_->reconfigure_ = true;
        frames.clear();
        res = impl_->EncodeHardware(raw, frames);
        if (res == AMF_OK) return 0;
      }
      LOG_WARN(
          "AMD AMF hardware HEVC encoding failed ({}); falling back to WMF "
          "HEVC",
          static_cast<int>(res));
      impl_->Reset(true);
      if (WmfHevcEncoder::IsSupported()) {
        auto wmf = std::make_unique<WmfHevcEncoder>(impl_->clock_);
        if (wmf->Init(impl_->config_) == 0) {
          impl_->fallback_ = std::move(wmf);
        }
      }
      if (!impl_->fallback_) {
        return -1;
      }
    }
    RawFrame fallback_raw = raw;
    if (fallback_raw.NativeFrame() &&
        !impl_->fallback_->SupportsNativeFrameInput(
            fallback_raw.NativeFrame()->type) &&
        !fallback_raw.MaterializeNativeFrame()) {
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

int AmfHevcEncoder::ForceIdr() {
  return impl_->worker_.Call([&] {
    impl_->force_idr_ = true;
    return impl_->fallback_ ? impl_->fallback_->ForceIdr() : 0;
  });
}

int AmfHevcEncoder::SetTargetBitrate(int bitrate) {
  if (bitrate <= 0) return -1;
  return impl_->worker_.Call([&] {
    bitrate = ClampEncoderTargetBitrate(bitrate, impl_->config_.max_bitrate);
    const int min_supported = (std::min)(10000, impl_->config_.max_bitrate);
    bitrate = (std::max)(bitrate, min_supported);
    if (impl_->fallback_) {
      const int result = impl_->fallback_->SetTargetBitrate(bitrate);
      if (result == 0) impl_->config_.init_bitrate = bitrate;
      return result;
    }
    if (!impl_->encoder_) return -1;
    const int applied = impl_->config_.init_bitrate;
    if (bitrate == applied) return 0;
    const bool at_bound =
        bitrate == min_supported || bitrate == impl_->config_.max_bitrate;
    if (!at_bound && applied > 0 &&
        std::llabs(static_cast<int64_t>(bitrate) - applied) * 5 < applied) {
      return 0;
    }
    impl_->config_.init_bitrate = bitrate;
    impl_->ApplyRateControlProperties(false);
    return 0;
  });
}

int AmfHevcEncoder::SetTargetFrameRate(int frame_rate) {
  if (frame_rate <= 0 || frame_rate > 240) return -1;
  return impl_->worker_.Call([&] {
    if (impl_->fallback_) {
      const int result = impl_->fallback_->SetTargetFrameRate(frame_rate);
      if (result == 0) impl_->config_.max_frame_rate = frame_rate;
      return result;
    }
    if (!impl_->encoder_) return -1;
    if (impl_->config_.max_frame_rate != frame_rate) {
      impl_->config_.max_frame_rate = frame_rate;
      impl_->encoder_->SetProperty(
          AMF_VIDEO_ENCODER_HEVC_FRAMERATE,
          AMFConstructRate(static_cast<amf_uint32>(frame_rate), 1));
      impl_->ApplyRateControlProperties(false);
    }
    return 0;
  });
}

int AmfHevcEncoder::GetResolution(int* width, int* height) const {
  if (!width || !height) return -1;
  return impl_->worker_.Call([&] {
    if (impl_->fallback_) return impl_->fallback_->GetResolution(width, height);
    if (!impl_->encoder_) return -1;
    *width = impl_->source_width_;
    *height = impl_->source_height_;
    return 0;
  });
}

std::string AmfHevcEncoder::GetEncoderName() const {
  return impl_->worker_.Call([&]() -> std::string {
    return impl_->fallback_ ? impl_->fallback_->GetEncoderName()
                            : "AMD AMF (HEVC, HW)";
  });
}

}  // namespace minirtc
