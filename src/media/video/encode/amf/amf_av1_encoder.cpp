#include "amf_av1_encoder.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <d3d11.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

#include "av1_encode_helpers.h"
#include "avt/svt_av1_encoder.h"
#include "components/Component.h"
#include "components/ComponentCaps.h"
#include "components/VideoEncoderAV1.h"
#include "core/Buffer.h"
#include "core/Context.h"
#include "core/Factory.h"
#include "core/Surface.h"
#include "log.h"
#include "wmf/wmf_av1_encoder.h"
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
                     const T& value) {
  if (!storage) return;
  const AMF_RESULT res = storage->SetProperty(name, value);
  if (res != AMF_OK) {
    LOG_DEBUG("AMF AV1 optional property failed: res={}",
              static_cast<int>(res));
  }
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

}  // namespace

class AmfAv1Encoder::Impl {
 public:
  explicit Impl(std::shared_ptr<SystemClock> clock)
      : clock_(std::move(clock)) {}

  ~Impl() {
    callback_worker_.Flush();
    worker_.Call([this] {
      Reset();
      fallback_.reset();
    });
  }

  void Reset() {
    if (encoder_) {
      encoder_->Terminate();
      encoder_ = nullptr;
    }
    if (context_) {
      context_->Terminate();
      context_ = nullptr;
    }
    factory_ = nullptr;
    if (amf_dll_) {
      FreeLibrary(amf_dll_);
      amf_dll_ = nullptr;
    }
    d3d_device_ = nullptr;
    d3d_tex_width_ = 0;
    d3d_tex_height_ = 0;
    using_d3d11_ = false;
    sequence_header_.clear();
    sequence_ = 0;
    force_idr_ = true;
    stats_.Reset();
  }

  void ApplyRateControlProperties(bool initial_config) {
    if (!encoder_) return;
    const amf_int64 fps = (std::max)(1, config_.max_frame_rate);
    const amf_int64 target_bps = (std::max)(10000, config_.init_bitrate);
    const amf_int64 peak_bps = (std::max)(
        target_bps,
        (std::min)(static_cast<amf_int64>(config_.max_bitrate),
                   target_bps + (target_bps / 2)));
    // Cap per-frame compressed bursts (1.2 Mbit..3.6 Mbit == 150 KB..450 KB)
    // to prevent desktop motion/IDR spikes from saturating UDP queues.
    const amf_int64 raw_frame_cap = (peak_bps * 3) / fps;
    const amf_int64 max_frame_bits =
        (std::max)(static_cast<amf_int64>(1'200'000LL),
                   (std::min)(static_cast<amf_int64>(3'600'000LL),
                              raw_frame_cap));

    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_TARGET_BITRATE, target_bps);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_PEAK_BITRATE, peak_bps);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_MAX_COMPRESSED_FRAME_SIZE,
                    max_frame_bits);

    if (!initial_config) {
      return;
    }

    // Keep VBV shallow (250ms at target bitrate, floor 2 Mbit) without rigid
    // HRD enforcement so static-to-motion transitions recover without VCN stalls.
    const amf_int64 vbv_bits =
        (std::max)(target_bps / 4, static_cast<amf_int64>(2'000'000LL));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_VBV_BUFFER_SIZE, vbv_bits);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_ENFORCE_HRD, false);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_FILLER_DATA, false);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_RATE_CONTROL_SKIP_FRAME,
                    false);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_MIN_Q_INDEX_INTRA,
                    static_cast<amf_int64>(20));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_MIN_Q_INDEX_INTER,
                    static_cast<amf_int64>(20));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_MAX_Q_INDEX_INTRA,
                    static_cast<amf_int64>(210));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_MAX_Q_INDEX_INTER,
                    static_cast<amf_int64>(210));
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

    res = factory_->CreateComponent(context_, AMFVideoEncoder_AV1, &encoder_);
    if (res != AMF_OK || !encoder_) return res != AMF_OK ? res : AMF_FAIL;

    if (!IsHardwareAccelerated(encoder_)) {
      return AMF_NOT_SUPPORTED;
    }

    const amf_uint32 fps =
        static_cast<amf_uint32>((std::max)(1, config_.max_frame_rate));

    res = encoder_->SetProperty(
        AMF_VIDEO_ENCODER_AV1_USAGE,
        static_cast<amf_int64>(AMF_VIDEO_ENCODER_AV1_USAGE_ULTRA_LOW_LATENCY));
    if (res != AMF_OK) {
      SetOptionalProp(
          encoder_, AMF_VIDEO_ENCODER_AV1_USAGE,
          static_cast<amf_int64>(AMF_VIDEO_ENCODER_AV1_USAGE_LOW_LATENCY));
    }

    SetOptionalProp(
        encoder_, AMF_VIDEO_ENCODER_AV1_ENCODING_LATENCY_MODE,
        static_cast<amf_int64>(
            AMF_VIDEO_ENCODER_AV1_ENCODING_LATENCY_MODE_LOWEST_LATENCY));
    SetOptionalProp(
        encoder_, AMF_VIDEO_ENCODER_AV1_QUALITY_PRESET,
        static_cast<amf_int64>(AMF_VIDEO_ENCODER_AV1_QUALITY_PRESET_SPEED));
    SetOptionalProp(
        encoder_, AMF_VIDEO_ENCODER_AV1_ALIGNMENT_MODE,
        static_cast<amf_int64>(
            AMF_VIDEO_ENCODER_AV1_ALIGNMENT_MODE_NO_RESTRICTIONS));
    SetOptionalProp(
        encoder_, AMF_VIDEO_ENCODER_AV1_PROFILE,
        static_cast<amf_int64>(AMF_VIDEO_ENCODER_AV1_PROFILE_MAIN));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_FRAMESIZE,
                    AMFConstructSize(width, height));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_FRAMERATE,
                    AMFConstructRate(fps, 1));
    SetOptionalProp(
        encoder_, AMF_VIDEO_ENCODER_AV1_RATE_CONTROL_METHOD,
        static_cast<amf_int64>(
            AMF_VIDEO_ENCODER_AV1_RATE_CONTROL_METHOD_LATENCY_CONSTRAINED_VBR));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_PRE_ANALYSIS_ENABLE, false);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_RATE_CONTROL_PREENCODE,
                    false);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_MAX_CONSECUTIVE_BPICTURES,
                    static_cast<amf_int64>(0));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_MAX_NUM_REFRAMES,
                    static_cast<amf_int64>(1));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_GOP_SIZE,
                    static_cast<amf_int64>(config_.key_frame_interval));
    SetOptionalProp(
        encoder_, AMF_VIDEO_ENCODER_AV1_HEADER_INSERTION_MODE,
        static_cast<amf_int64>(
            AMF_VIDEO_ENCODER_AV1_HEADER_INSERTION_MODE_KEY_FRAME_ALIGNED));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_SCREEN_CONTENT_TOOLS,
                    true);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_PALETTE_MODE, true);
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_TILES_PER_FRAME,
                    static_cast<amf_int64>(4));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_INPUT_QUEUE_SIZE,
                    static_cast<amf_int64>(4));
    SetOptionalProp(encoder_, AMF_VIDEO_ENCODER_AV1_QUERY_TIMEOUT,
                    static_cast<amf_int64>(50));

    ApplyRateControlProperties(true);

    res = encoder_->Init(amf::AMF_SURFACE_NV12, width, height);
    if (res != AMF_OK) return res;

    CacheSequenceHeaderFromExtraData(sequence_header_);
    return AMF_OK;
  }

  void CacheSequenceHeaderFromExtraData(std::vector<uint8_t>& out_seq_hdr) {
    if (!encoder_) return;
    amf::AMFVariant extra_var;
    if (encoder_->GetProperty(AMF_VIDEO_ENCODER_AV1_EXTRA_DATA, &extra_var) ==
            AMF_OK &&
        extra_var.type == amf::AMF_VARIANT_INTERFACE &&
        extra_var.pInterface != nullptr) {
      amf::AMFBufferPtr extra_buf(extra_var.pInterface);
      if (extra_buf && extra_buf->GetNative() && extra_buf->GetSize() > 0) {
        CacheSequenceHeaderFromBytes(
            static_cast<const uint8_t*>(extra_buf->GetNative()),
            extra_buf->GetSize(), out_seq_hdr);
      }
    }
  }

  AMF_RESULT Initialize(ID3D11Device* d3d_device, int d3d_tex_w,
                        int d3d_tex_h) {
    Reset();
    if (FAILED(worker_.Status())) return AMF_FAIL;

    const auto candidates =
        BuildCandidateEncodeSizes(source_width_, source_height_);
    AMF_RESULT last_res = AMF_FAIL;

    if (d3d_device && d3d_tex_w > 0 && d3d_tex_h > 0) {
      last_res = Configure(d3d_tex_w, d3d_tex_h, d3d_device);
      if (last_res == AMF_OK) {
        d3d_device_ = d3d_device;
        d3d_tex_width_ = d3d_tex_w;
        d3d_tex_height_ = d3d_tex_h;
        d3d11_failed_device_ = nullptr;
        encode_width_ = d3d_tex_w;
        encode_height_ = d3d_tex_h;
        encode_mode_ = (d3d_tex_w != source_width_ || d3d_tex_h != source_height_)
                           ? EncodeMode::kPad
                           : EncodeMode::kNative;
      } else {
        LOG_WARN(
            "AMD AMF AV1 D3D11 config {}x{} failed (res={}); falling back to "
            "CPU surface input",
            d3d_tex_w, d3d_tex_h, static_cast<int>(last_res));
        d3d11_failed_device_ = d3d_device;
        Reset();
      }
    }

    if (!encoder_) {
      for (const auto& candidate : candidates) {
        last_res = Configure(candidate.width, candidate.height, nullptr);
        if (last_res == AMF_OK) {
          encode_width_ = candidate.width;
          encode_height_ = candidate.height;
          encode_mode_ = candidate.mode;
          break;
        }
        LOG_DEBUG("AMD AMF AV1 config {}x{} failed: res={}", candidate.width,
                  candidate.height, static_cast<int>(last_res));
        Reset();
      }
    }

    if (!encoder_) {
      LOG_WARN("AMD AMF AV1 encoder initialization failed: res={}",
               static_cast<int>(last_res));
      return last_res;
    }

    LOG_INFO(
        "AMD AMF hardware AV1 encoder initialized ({}): {}x{} target={}bps "
        "fps={} vbr_latency+hrd",
        using_d3d11_ ? "D3D11 zero-copy" : "CPU surface", encode_width_,
        encode_height_, config_.init_bitrate, config_.max_frame_rate);
    if (encode_mode_ == EncodeMode::kPad) {
      LOG_INFO(
          "AMD AMF AV1 encoder padding {}x{} -> {}x{} (1:1 crop metadata)",
          source_width_, source_height_, encode_width_, encode_height_);
    } else if (encode_mode_ == EncodeMode::kScale) {
      LOG_INFO("AMD AMF AV1 encoder scaling {}x{} -> {}x{}", source_width_,
               source_height_, encode_width_, encode_height_);
    }
    reconfigure_ = false;
    return AMF_OK;
  }

  int VisibleWidth() const {
    return encode_mode_ == EncodeMode::kScale ? encode_width_ : source_width_;
  }
  int VisibleHeight() const {
    return encode_mode_ == EncodeMode::kScale ? encode_height_ : source_height_;
  }

  AMF_RESULT PrepareInputSurface(const RawFrame& raw, bool is_d3d11_input,
                                 amf::AMFSurfacePtr& out_surface) {
    if (!context_) return AMF_NOT_INITIALIZED;
    AMF_RESULT res = AMF_OK;
    if (is_d3d11_input && using_d3d11_) {
      const MiniRtcNativeVideoFrame* native = raw.NativeFrame();
      res = context_->CreateSurfaceFromDX11Native(
          native->payload.d3d11_nv12.texture, &out_surface, nullptr);
      if (res != AMF_OK || !out_surface) {
        d3d11_failed_device_ = d3d_device_;
        return res != AMF_OK ? res : AMF_FAIL;
      }
    } else {
      const uint8_t* src_nv12 = ResolveCpuNv12(raw, cpu_materialize_buf_);
      if (!src_nv12) return AMF_INVALID_ARG;
      res = context_->AllocSurface(amf::AMF_MEMORY_HOST, amf::AMF_SURFACE_NV12,
                                   encode_width_, encode_height_, &out_surface);
      if (res != AMF_OK || !out_surface) {
        return res != AMF_OK ? res : AMF_FAIL;
      }
      amf::AMFPlane* plane_y = out_surface->GetPlaneAt(0);
      amf::AMFPlane* plane_uv = out_surface->GetPlaneAt(1);
      if (!plane_y || !plane_uv || !plane_y->GetNative() ||
          !plane_uv->GetNative()) {
        return AMF_FAIL;
      }
      if (!PrepareNv12ToStridedPlanes(
              src_nv12, source_width_, source_height_,
              static_cast<uint8_t*>(plane_y->GetNative()), plane_y->GetHPitch(),
              static_cast<uint8_t*>(plane_uv->GetNative()),
              plane_uv->GetHPitch(), encode_width_, encode_height_,
              encode_mode_, &scaler_scratch_)) {
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

    bool hint_key = false;
    amf::AMFVariant ft_var;
    if (out_buf->GetProperty(AMF_VIDEO_ENCODER_AV1_OUTPUT_FRAME_TYPE,
                             &ft_var) == AMF_OK &&
        ft_var.type == amf::AMF_VARIANT_INT64) {
      hint_key =
          (ft_var.int64Value == AMF_VIDEO_ENCODER_AV1_OUTPUT_FRAME_TYPE_KEY);
    }

    const auto processed = ProcessEncodedAv1Bitstream(
        output_bytes_, sequence_header_, hint_key, encode_mode_, source_width_,
        source_height_,
        [this](std::vector<uint8_t>& out_seq_hdr) {
          CacheSequenceHeaderFromExtraData(out_seq_hdr);
        },
        "AMF AV1");
    if (!processed.valid) return AMF_FAIL;

    const uint32_t out_w = static_cast<uint32_t>(VisibleWidth());
    const uint32_t out_h = static_cast<uint32_t>(VisibleHeight());
    PushEncodedAv1Frame(output_bytes_, out_w, out_h, processed.is_keyframe,
                        raw.CapturedTimestamp(), clock_->CurrentTime(), frames);
    if (processed.is_keyframe) {
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
        native->payload.d3d11_nv12.subresource_index == 0 &&
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
      const bool dev_changed = dev != d3d_device_;
      const bool tex_changed = tex_w != d3d_tex_width_ || tex_h != d3d_tex_height_;
      if (!encoder_ || size_changed || switched_d3d11 || dev_changed ||
          tex_changed || reconfigure_) {
        if (sequence_ > 0) {
          LOG_INFO(
              "Reinitializing AMD AMF AV1 encoder (D3D11): size_chg={} "
              "to_d3d11={} dev_chg={} tex_chg={} reconfig={} {}x{}->{}x{}",
              size_changed, switched_d3d11, dev_changed, tex_changed,
              reconfigure_, source_width_, source_height_, raw.Width(),
              raw.Height());
        }
        source_width_ = static_cast<int>(raw.Width());
        source_height_ = static_cast<int>(raw.Height());
        config_.init_width = source_width_;
        config_.init_height = source_height_;
        const AMF_RESULT res = Initialize(dev, tex_w, tex_h);
        if (res != AMF_OK) return res;
      }
    } else {
      const bool size_changed = static_cast<int>(raw.Width()) != source_width_ ||
                                static_cast<int>(raw.Height()) != source_height_;
      const bool switched_cpu = using_d3d11_;
      if (!encoder_ || size_changed || switched_cpu || reconfigure_) {
        if (sequence_ > 0) {
          LOG_INFO(
              "Reinitializing AMD AMF AV1 encoder (CPU): size_chg={} to_cpu={} "
              "reconfig={} {}x{}->{}x{}",
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

    amf::AMFSurfacePtr surface;
    AMF_RESULT res = PrepareInputSurface(raw, is_d3d11_input, surface);
    if (res != AMF_OK) return res;

    const amf_pts duration = 10'000'000LL / config_.max_frame_rate;
    surface->SetPts(next_time_);
    surface->SetDuration(duration);
    next_time_ += duration;

    if (sequence_ % config_.key_frame_interval == 0) {
      force_idr_ = true;
    }
    if (force_idr_) {
      SetOptionalProp(
          surface, AMF_VIDEO_ENCODER_AV1_FORCE_FRAME_TYPE,
          static_cast<amf_int64>(AMF_VIDEO_ENCODER_AV1_FORCE_FRAME_TYPE_KEY));
      SetOptionalProp(surface,
                      AMF_VIDEO_ENCODER_AV1_FORCE_INSERT_SEQUENCE_HEADER, true);
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
        if (res == AMF_OK) {
          submitted = true;
          encode_submitted = sub_end;
          surface = nullptr;
        } else if (res != AMF_INPUT_FULL && res != AMF_NEED_MORE_INPUT) {
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
        const auto encode_end = std::chrono::steady_clock::now();
        const double total_ms =
            std::chrono::duration<double, std::milli>(encode_end - encode_start)
                .count();
        const double hw_ms =
            submitted ? std::chrono::duration<double, std::milli>(
                            encode_end - encode_submitted)
                            .count()
                      : 0.0;
        const size_t encoded_bytes = frames.empty() ? 0 : frames.back().Size();
        const bool is_key =
            !frames.empty() &&
            frames.back().FrameType() == VideoFrameType::kVideoFrameKey;
        stats_.Record("AMF AV1", encode_end, encode_width_, encode_height_,
                      using_d3d11_, total_ms, wait_poll_ms, submit_ms, hw_ms,
                      encoded_bytes, is_key);
        return AMF_OK;
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
  ID3D11Device* d3d_device_ = nullptr;
  ID3D11Device* d3d11_failed_device_ = nullptr;
  bool using_d3d11_ = false;
  int d3d_tex_width_ = 0;
  int d3d_tex_height_ = 0;
  bool force_idr_ = true;
  bool reconfigure_ = false;
  bool initialized_ = false;
  int source_width_ = 0;
  int source_height_ = 0;
  int encode_width_ = 0;
  int encode_height_ = 0;
  EncodeMode encode_mode_ = EncodeMode::kNative;
  uint64_t sequence_ = 0;
  amf_pts next_time_ = 0;
  std::vector<uint8_t> sequence_header_;
  std::vector<uint8_t> output_bytes_;
  std::vector<uint8_t> cpu_materialize_buf_;
  std::vector<uint8_t> scaler_scratch_;
  std::unique_ptr<MediaCodec> fallback_;
  EncodeStatsTracker stats_;
};

AmfAv1Encoder::AmfAv1Encoder(std::shared_ptr<SystemClock> clock)
    : impl_(std::make_unique<Impl>(std::move(clock))) {}

AmfAv1Encoder::~AmfAv1Encoder() = default;

bool AmfAv1Encoder::IsSupported() {
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
        factory->CreateComponent(context, AMFVideoEncoder_AV1, &encoder) ==
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

int AmfAv1Encoder::Init(const MediaCodecConfig& config) {
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
    impl_->initialized_ =
        (impl_->Initialize(nullptr, 0, 0) == AMF_OK);
    return impl_->initialized_ ? 0 : -1;
  });
}

bool AmfAv1Encoder::SupportsNativeFrameInput(
    MiniRtcNativeVideoFrameType frame_type) const {
  return frame_type == MiniRtcNativeVideoFrameD3d11Nv12 ||
         frame_type == MiniRtcNativeVideoFrameCpuNv12;
}

int AmfAv1Encoder::Encode(const RawFrame& raw,
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
      const AMF_RESULT res = impl_->EncodeHardware(raw, frames);
      if (res == AMF_OK) return 0;
      LOG_WARN(
          "AMD AMF hardware AV1 encoding failed ({}); falling back to WMF/"
          "SVT-AV1",
          static_cast<int>(res));
      impl_->Reset();
      if (WmfAv1Encoder::IsSupported()) {
        auto wmf = std::make_unique<WmfAv1Encoder>(impl_->clock_);
        if (wmf->Init(impl_->config_) == 0) {
          impl_->fallback_ = std::move(wmf);
        }
      }
      if (!impl_->fallback_) {
        auto svt = std::make_unique<SvtAv1Encoder>(impl_->clock_);
        if (svt->Init(impl_->config_) == 0) {
          impl_->fallback_ = std::move(svt);
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

int AmfAv1Encoder::ForceIdr() {
  return impl_->worker_.Call([&] {
    impl_->force_idr_ = true;
    return impl_->fallback_ ? impl_->fallback_->ForceIdr() : 0;
  });
}

int AmfAv1Encoder::SetTargetBitrate(int bitrate) {
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

int AmfAv1Encoder::SetTargetFrameRate(int frame_rate) {
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
          AMF_VIDEO_ENCODER_AV1_FRAMERATE,
          AMFConstructRate(static_cast<amf_uint32>(frame_rate), 1));
      impl_->ApplyRateControlProperties(false);
    }
    return 0;
  });
}

int AmfAv1Encoder::GetResolution(int* width, int* height) const {
  if (!width || !height) return -1;
  return impl_->worker_.Call([&] {
    if (impl_->fallback_) return impl_->fallback_->GetResolution(width, height);
    if (!impl_->encoder_) return -1;
    *width = impl_->VisibleWidth();
    *height = impl_->VisibleHeight();
    return 0;
  });
}

std::string AmfAv1Encoder::GetEncoderName() const {
  return impl_->worker_.Call([&]() -> std::string {
    return impl_->fallback_ ? impl_->fallback_->GetEncoderName()
                            : "AMD AMF (AV1, HW)";
  });
}

}  // namespace minirtc
