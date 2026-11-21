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

// Per-stream capture admission and delay-based screen frame-rate pacing.
// Keep phase across small capture jitter, do not accumulate a burst of
// overdue frames after capture pauses, and under congestion pace down the
// admitted frame rate evenly instead of dropping a contiguous batch of frames.
class VideoFrameCadence {
 public:
  // Without network delay or loss evidence, allow up to 240 ms of pacer queue
  // (and 280 ms oldest packet wait) so scene-change bursts drain without
  // dropping a batch of frames. When RTT excess, heavy packet loss or
  // congestion confirms network pressure, tighten the hard ceiling to 160 ms.
  // Ordinary Wi-Fi random loss (a few percent) is repaired by FEC/NACK and
  // must not lock the stream to a lower frame rate by itself.
  static constexpr int64_t kRelaxedAdmissionQueueMs = 240;
  static constexpr int64_t kCongestedAdmissionQueueMs = 160;
  static constexpr int64_t kHardOldestPacketWaitMs = 280;
  static constexpr int kMinScreenFrameRate = 15;
  static constexpr int kPressureLossPercent = 10;
  static constexpr int kSevereLossPercent = 20;

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

  // Delay- and queue-aware screen frame admission called after Accept().
  // Returns false when the frame is paced out by smooth frame-rate step-down
  // or blocked by the hard queue safety valve.
  bool AdmitScreenFrame(int64_t now_us, int max_frame_rate,
                        int64_t expected_queue_ms, int64_t oldest_wait_ms,
                        int64_t rtt_ms, int loss_pct, bool congested) {
    UpdateRttBaseline(now_us, rtt_ms);
    const int64_t rtt_excess_ms = RttExcessMs(rtt_ms);
    const bool network_pressured = congested ||
                                   loss_pct >= kPressureLossPercent ||
                                   rtt_excess_ms >= 60;
    const int64_t hard_queue_limit_ms = network_pressured
                                            ? kCongestedAdmissionQueueMs
                                            : kRelaxedAdmissionQueueMs;
    if (expected_queue_ms >= hard_queue_limit_ms ||
        oldest_wait_ms >= kHardOldestPacketWaitMs) {
      return false;
    }

    int desired_level = 0;
    if (!network_pressured) {
      // No heavy loss and no RTT build-up: let normal encoder bursts
      // (<160 ms) pass at full frame rate, and gently halve the rate in the
      // 160-240 ms band.
      if (expected_queue_ms >= kCongestedAdmissionQueueMs) {
        desired_level = 2;
      }
    } else if (rtt_excess_ms >= 150 || loss_pct >= kSevereLossPercent ||
               expected_queue_ms >= 130) {
      desired_level = 3;
    } else if (rtt_excess_ms >= 100 || loss_pct >= kPressureLossPercent ||
               expected_queue_ms >= 90 || congested) {
      desired_level = 2;
    } else if (rtt_excess_ms >= 60 || expected_queue_ms >= 60) {
      desired_level = 1;
    }

    if (desired_level > pressure_level_) {
      pressure_level_ = desired_level;
      hold_level_until_us_ = now_us + kStepHoldUs;
    } else if (desired_level == pressure_level_ && desired_level > 0) {
      hold_level_until_us_ = now_us + kStepHoldUs;
    } else if (desired_level < pressure_level_ &&
               now_us >= hold_level_until_us_) {
      --pressure_level_;
      hold_level_until_us_ =
          pressure_level_ > 0 ? now_us + kStepHoldUs : 0;
    }

    const int max_fps = std::max(1, max_frame_rate);
    const int effective_fps = EffectiveFrameRateForLevel(max_fps,
                                                         pressure_level_);
    if (effective_fps >= max_fps) {
      next_admitted_us_ = 0;
      last_admitted_us_ = now_us;
      return true;
    }

    const int64_t interval_us = 1000000 / effective_fps;
    const int64_t tolerance_us = std::min<int64_t>(4000, interval_us / 4);
    if (next_admitted_us_ == 0 && last_admitted_us_ > 0) {
      next_admitted_us_ = last_admitted_us_ + interval_us;
    }
    if (now_us + tolerance_us < next_admitted_us_) {
      return false;
    }
    next_admitted_us_ =
        next_admitted_us_ == 0
            ? now_us + interval_us
            : std::max(next_admitted_us_ + interval_us,
                       now_us + interval_us / 2);
    last_admitted_us_ = now_us;
    return true;
  }

  int EffectiveFrameRate(int max_frame_rate) const {
    return EffectiveFrameRateForLevel(std::max(1, max_frame_rate),
                                      pressure_level_);
  }

 private:
  static constexpr int64_t kRttBucketUs = 10'000'000;
  static constexpr int64_t kStepHoldUs = 300'000;
  static constexpr int64_t kUnsetRttMs = INT64_MAX;

  static int EffectiveFrameRateForLevel(int max_fps, int level) {
    const int floor_fps = std::min(max_fps, kMinScreenFrameRate);
    switch (level) {
      case 1:
        return std::max(floor_fps, (max_fps * 3 + 2) / 4);
      case 2:
        return std::max(floor_fps, max_fps / 2);
      case 3:
        return std::max(floor_fps, max_fps / 4);
      default:
        return max_fps;
    }
  }

  void UpdateRttBaseline(int64_t now_us, int64_t rtt_ms) {
    if (rtt_ms <= 0) return;
    if (rtt_bucket_start_us_ == 0 ||
        now_us - rtt_bucket_start_us_ >= kRttBucketUs) {
      rtt_min_prev_ms_ = rtt_min_current_ms_;
      rtt_min_current_ms_ = kUnsetRttMs;
      rtt_bucket_start_us_ = now_us;
    }
    rtt_min_current_ms_ = std::min(rtt_min_current_ms_, rtt_ms);
  }

  int64_t RttExcessMs(int64_t rtt_ms) const {
    const int64_t baseline_ms = std::min(rtt_min_current_ms_, rtt_min_prev_ms_);
    if (rtt_ms <= 0 || baseline_ms == kUnsetRttMs) return 0;
    return std::max<int64_t>(0, rtt_ms - baseline_ms);
  }

  int64_t next_frame_us_ = 0;
  int64_t next_admitted_us_ = 0;
  int64_t last_admitted_us_ = 0;
  int64_t hold_level_until_us_ = 0;
  int pressure_level_ = 0;
  int64_t rtt_min_current_ms_ = kUnsetRttMs;
  int64_t rtt_min_prev_ms_ = kUnsetRttMs;
  int64_t rtt_bucket_start_us_ = 0;
};
}  // namespace minirtc

#endif