/*
 * Copyright (c) 2026 MiniRTC Contributors, All Rights Reserved.
 */

#ifndef _D3D11_HEVC_DECODER_H_
#define _D3D11_HEVC_DECODER_H_

#include <memory>
#include <string>

#include "media_codec.h"

namespace minirtc {

// Windows Media Foundation HEVC (H.265) decoder MFT (Microsoft HEVC Video
// Extensions) with D3D11 DXVA hardware acceleration and automatic CPU fallback
// when the GPU does not expose HEVC VLD Main.
class D3D11HevcDecoder final : public MediaCodec {
 public:
  explicit D3D11HevcDecoder(std::shared_ptr<SystemClock> clock,
                            bool native_video_output = false);
  ~D3D11HevcDecoder() override;

  static bool IsSupported();

  int Init() override;
  int Decode(std::unique_ptr<ReceivedFrame> frame,
             std::function<void(const DecodedFrame*)> callback) override;
  std::string GetDecoderName() const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace minirtc

#endif
