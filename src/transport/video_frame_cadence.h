/*
 * @Author: DI JUNKUN
 * @Date: 2026-09-24
 * Copyright (c) 2026 by DI JUNKUN, All Rights Reserved.
 */

#ifndef _VIDEO_FRAME_CADENCE_H_
#define _VIDEO_FRAME_CADENCE_H_

#include <algorithm>
#include <cstdint>

namespace minirtc {

// Per-stream capture admission. Keep phase across small capture jitter, but
// do not accumulate a burst of overdue frames after capture pauses.
class VideoFrameCadence {
 public:
  bool Accept(int64_t now_us, int frame_rate) {
    const int64_t interval_us = 1000000 / std::max(1, frame_rate);
    const int64_t tolerance_us = std::min<int64_t>(4000, interval_us / 4);
    if (now_us + tolerance_us < next_frame_us_) return false;
    next_frame_us_ =
        next_frame_us_ == 0
            ? now_us + interval_us
            : std::max(next_frame_us_ + interval_us, now_us + interval_us / 2);
    return true;
  }

 private:
  int64_t next_frame_us_ = 0;
};

}  // namespace minirtc

#endif