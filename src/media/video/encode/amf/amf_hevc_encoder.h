/*
 * Copyright (c) 2026 CrossDesk / MiniRTC Contributors
 *
 * AMD AMF HEVC (H.265) hardware encoder.
 */

#ifndef _AMF_HEVC_ENCODER_H_
#define _AMF_HEVC_ENCODER_H_

#include <memory>
#include <string>

#include "media_codec.h"

namespace minirtc {

class AmfHevcEncoder : public MediaCodec {
 public:
  static bool IsSupported();

  explicit AmfHevcEncoder(std::shared_ptr<SystemClock> clock);
  ~AmfHevcEncoder() override;

  int Init(const MediaCodecConfig& config) override;
  int Encode(const RawFrame& raw_frame,
             std::function<int(const EncodedFrame&)> on_encoded_image) override;
  int ForceIdr() override;
  int SetTargetBitrate(int bitrate) override;
  int SetTargetFrameRate(int frame_rate) override;
  int GetResolution(int* width, int* height) const override;
  std::string GetEncoderName() const override;
  bool SupportsNativeFrameInput(
      MiniRtcNativeVideoFrameType frame_type) const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace minirtc

#endif  // _AMF_HEVC_ENCODER_H_
