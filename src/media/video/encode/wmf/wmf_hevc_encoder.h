/*
 * Copyright (c) 2026 MiniRTC Contributors, All Rights Reserved.
 */

#ifndef _WMF_HEVC_ENCODER_H_
#define _WMF_HEVC_ENCODER_H_

#include <memory>
#include <string>

#include "media_codec.h"

namespace minirtc {

// Windows Media Foundation hardware HEVC (H.265) encoder MFT.
// Supports zero-copy D3D11 NV12 textures as well as CPU NV12 buffers.
class WmfHevcEncoder final : public MediaCodec {
 public:
  explicit WmfHevcEncoder(std::shared_ptr<SystemClock> clock);
  ~WmfHevcEncoder() override;

  static bool IsSupported();

  int Init(const MediaCodecConfig& config) override;
  int Encode(const RawFrame& frame,
             std::function<int(const EncodedFrame&)> callback) override;
  int ForceIdr() override;
  int SetTargetBitrate(int bitrate) override;
  int SetTargetFrameRate(int frame_rate) override;
  bool SupportsDynamicEncodingSpeedPriority() const override { return true; }
  int SetPrioritizeEncodingSpeedOverQuality(bool prioritize_speed) override;
  bool SupportsNativeFrameInput(
      MiniRtcNativeVideoFrameType frame_type) const override;
  int GetResolution(int* width, int* height) const override;
  std::string GetEncoderName() const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace minirtc

#endif
