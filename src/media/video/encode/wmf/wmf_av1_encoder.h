#ifndef _WMF_AV1_ENCODER_H_
#define _WMF_AV1_ENCODER_H_

#include "media_codec.h"

namespace minirtc {

// Uses hardware AV1 MFTs registered by the graphics driver via Windows Media
// Foundation (e.g., AMD AMF / Intel QSV / NVIDIA AV1 MFT). Input is packed
// NV12; output is raw AV1 OBU bitstream. Falls back to SVT-AV1 on failure.
class WmfAv1Encoder final : public MediaCodec {
 public:
  explicit WmfAv1Encoder(std::shared_ptr<SystemClock> clock);
  ~WmfAv1Encoder() override;
  static bool IsSupported();
  int Init(const MediaCodecConfig& config) override;
  int Encode(const RawFrame& frame,
             std::function<int(const EncodedFrame&)> callback) override;
  int ForceIdr() override;
  int SetTargetBitrate(int bitrate) override;
  int SetTargetFrameRate(int frame_rate) override;
  int GetResolution(int* width, int* height) const override;
  std::string GetEncoderName() const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace minirtc
#endif
