#include "congestion_control.h"

#include <algorithm>
#include <numeric>
#include <vector>

#include "bitrate_limits.h"
#include "log.h"

namespace minirtc {
namespace {
constexpr int64_t kLossUpdateInterval = 1000;

// Pacing-rate relative to our target send rate.
// Multiplicative factor that is applied to the target bitrate to calculate
// the number of bytes that can be transmitted per interval.
// Increasing this factor will result in lower delays in cases of bitrate
// overshoots from the encoder.
constexpr float kDefaultPaceMultiplier = 2.5f;

// If the probe result is far below the current throughput estimate
// it's unlikely that the probe is accurate, so we don't want to drop too far.
// However, if we actually are overusing, we want to drop to something slightly
// below the current throughput estimate to drain the network queues.
constexpr double kProbeDropThroughputFraction = 0.85;

constexpr DataRate kDirectStartingRate = DataRate::BitsPerSec(2500000);
constexpr DataRate kRelayStartingRate = DataRate::BitsPerSec(1500000);
constexpr TimeDelta kProbeEstimateSettleTime = TimeDelta::Millis(100);

// Idle desktop hold (RustDesk-style: a static screen never lowers the rate).
// While media is application limited (ALR), GCC learns nothing about the
// link: its acknowledged throughput is the tiny idle traffic, so a single
// overuse sample (a scroll burst, Wi-Fi jitter) cuts the target to 0.85 x
// that throughput and nothing raises it again until motion resumes into a
// starved pacer. The hold keeps the last loaded-link target instead, and
// after motion resumes keeps it for a grace window so the first large frames
// drain at the pre-idle rate while fresh feedback confirms or refutes it.
constexpr TimeDelta kIdleHoldResumeGrace = TimeDelta::Seconds(3);
// Below this there is nothing worth protecting.
constexpr DataRate kIdleHoldMinRate = DataRate::BitsPerSec(1000000);
// Only confirmed congestion lowers the held rate. Like RustDesk's TestDelay,
// queueing delay above the path baseline counts, and one bad sample is
// jitter: kIdleHoldBadRttSamples consecutive fresh feedback RTTs must exceed
// the baseline, the first at least kIdleHoldCongestionConfirm ago. The
// trendline overuse state is deliberately not used: sparse idle traffic
// trips it without any queue. Loss above 10% (the level at which
// SendSideBandwidthEstimation itself decreases) in two consecutive windows
// of at least kIdleHoldCongestionConfirm and kIdleHoldMinLossPackets also
// counts.
constexpr TimeDelta kIdleHoldRttExcess = TimeDelta::Millis(150);
constexpr double kIdleHoldRttRatio = 2.0;
constexpr int kIdleHoldBadRttSamples = 3;
constexpr TimeDelta kIdleHoldCongestionConfirm = TimeDelta::Millis(500);
constexpr double kIdleHoldLossRatio = 0.10;
constexpr int64_t kIdleHoldMinLossPackets = 20;
constexpr int kIdleHoldBadLossWindows = 2;
// While idle, confirmed congestion lowers the held rate by RustDesk's
// confirmed-delay steps, at most once per kIdleHoldBackoffInterval.
constexpr TimeDelta kIdleHoldBackoffInterval = TimeDelta::Seconds(3);
// The RTT baseline is the minimum over the current and the previous bucket,
// i.e. over the last 10-20 s: it follows a path change but a single high
// sample never becomes the baseline.
constexpr TimeDelta kIdleHoldRttBucket = TimeDelta::Seconds(10);
// Feedback arrives every ~25-50 ms under load; a longer gap ends a high-RTT
// streak, and a streak older than this cannot confirm congestion.
constexpr TimeDelta kIdleHoldRttSampleGap = TimeDelta::Millis(250);

double IdleHoldBackoffFactor(TimeDelta rtt_excess, double loss_ratio) {
  double factor = 1.0;
  if (rtt_excess >= kIdleHoldRttExcess) {
    // RustDesk video_qos.rs ratio_reduction() for confirmed delay.
    if (rtt_excess < TimeDelta::Millis(200)) {
      factor = 0.95;
    } else if (rtt_excess < TimeDelta::Millis(300)) {
      factor = 0.9;
    } else if (rtt_excess < TimeDelta::Millis(500)) {
      factor = 0.7;
    } else {
      factor = 0.5;
    }
  }
  if (loss_ratio > kIdleHoldLossRatio) {
    // SendSideBandwidthEstimation's loss decrease: rate * (1 - 0.5 * loss).
    factor = std::min(factor, 1.0 - 0.5 * std::min(loss_ratio, 1.0));
  }
  return factor;
}

}  // namespace

BandwidthLimitedCause GetBandwidthLimitedCause(LossBasedState loss_based_state,
                                               bool is_rtt_above_limit,
                                               BandwidthUsage bandwidth_usage) {
  if (bandwidth_usage == BandwidthUsage::kBwOverusing ||
      bandwidth_usage == BandwidthUsage::kBwUnderusing) {
    return BandwidthLimitedCause::kDelayBasedLimitedDelayIncreased;
  } else if (is_rtt_above_limit) {
    return BandwidthLimitedCause::kRttBasedBackOffHighRtt;
  }
  switch (loss_based_state) {
    case LossBasedState::kDecreasing:
      // Probes may not be sent in this state.
      return BandwidthLimitedCause::kLossLimitedBwe;
    case webrtc::LossBasedState::kIncreaseUsingPadding:
      // Probes may not be sent in this state.
      return BandwidthLimitedCause::kLossLimitedBwe;
    case LossBasedState::kIncreasing:
      // Probes may be sent in this state.
      return BandwidthLimitedCause::kLossLimitedBweIncreasing;
    case LossBasedState::kDelayBasedEstimate:
      return BandwidthLimitedCause::kDelayBasedLimited;
    default:
      return BandwidthLimitedCause::kLossLimitedBwe;
  }
}

CongestionControl::CongestionControl()
    : packet_feedback_only_(true),
      use_min_allocatable_as_lower_bound_(false),
      ignore_probes_lower_than_network_estimate_(false),
      limit_probes_lower_than_throughput_estimate_(false),
      pace_at_max_of_bwe_and_lower_link_capacity_(false),
      limit_pacingfactor_by_upper_link_capacity_estimate_(false),
      probe_controller_(new ProbeController()),
      congestion_window_pushback_controller_(
          // std::make_unique<CongestionWindowPushbackController>()
          nullptr),
      bandwidth_estimation_(new SendSideBandwidthEstimation()),
      alr_detector_(new AlrDetector()),
      probe_bitrate_estimator_(new ProbeBitrateEstimator()),
      delay_based_bwe_(new DelayBasedBwe()),
      acknowledged_bitrate_estimator_(
          AcknowledgedBitrateEstimatorInterface::Create()),
      pacing_factor_(kDefaultPaceMultiplier),
      min_total_allocated_bitrate_(DataRate::Zero()),
      max_padding_rate_(DataRate::Zero()),
      last_loss_based_target_rate_(DataRate::Zero()) {
  NetworkControllerConfig config;

  config.constraints.at_time = Timestamp::PlusInfinity();
  config.constraints.min_data_rate = DataRate::BitsPerSec(300000);
  config.constraints.max_data_rate =
      DataRate::BitsPerSec(kDefaultMaxNetworkBitrateBps);
  config.constraints.starting_rate = kDirectStartingRate;

  config.stream_based_config.at_time = Timestamp::PlusInfinity();
  config.stream_based_config.requests_alr_probing = true;
  config.stream_based_config.enable_repeated_initial_probing = false;
  config.stream_based_config.pacing_factor = kDefaultPaceMultiplier;
  config.stream_based_config.min_total_allocated_bitrate = DataRate::Zero();
  config.stream_based_config.max_padding_rate = DataRate::Zero();
  config.stream_based_config.max_total_allocated_bitrate = DataRate::Zero();

  initial_config_ = config;
}

CongestionControl::~CongestionControl() {}

void CongestionControl::SetRepeatedInitialProbing(bool enable) {
  if (initial_config_) {
    initial_config_->stream_based_config.enable_repeated_initial_probing =
        enable;
  }
  probe_controller_->EnableRepeatedInitialProbing(enable);
}

void CongestionControl::SetIdleRateHold(bool enabled) {
  idle_rate_hold_enabled_ = enabled;
  if (!enabled) ResetIdleRateHold();
}

void CongestionControl::ResetIdleRateHold() {
  idle_hold_rate_ = DataRate::Zero();
  idle_hold_learned_ = false;
  idle_hold_until_ = Timestamp::MinusInfinity();
  idle_hold_in_alr_ = false;
  idle_hold_rtt_min_current_ = TimeDelta::PlusInfinity();
  idle_hold_rtt_min_previous_ = TimeDelta::PlusInfinity();
  idle_hold_rtt_bucket_start_ = Timestamp::MinusInfinity();
  idle_hold_bad_rtt_samples_ = 0;
  idle_hold_bad_rtt_since_ = Timestamp::PlusInfinity();
  idle_hold_bad_rtt_last_ = Timestamp::MinusInfinity();
  idle_hold_last_rtt_sample_ = Timestamp::MinusInfinity();
  idle_hold_loss_window_start_ = Timestamp::MinusInfinity();
  idle_hold_loss_packets_ = 0;
  idle_hold_lost_packets_ = 0;
  idle_hold_bad_loss_windows_ = 0;
  idle_hold_loss_ratio_ = 0.0;
  idle_hold_last_backoff_ = Timestamp::MinusInfinity();
}

void CongestionControl::UpdateIdleHoldFeedback(TimeDelta rtt, int64_t packets,
                                               int64_t lost, bool in_alr,
                                               bool delay_updated,
                                               Timestamp at_time) {
  if (!idle_rate_hold_enabled_) return;
  if (!in_alr && delay_updated) idle_hold_learned_ = true;
  if (rtt.IsFinite() && rtt > TimeDelta::Zero()) {
    if (at_time - idle_hold_rtt_bucket_start_ >= kIdleHoldRttBucket) {
      idle_hold_rtt_min_previous_ = idle_hold_rtt_min_current_;
      idle_hold_rtt_min_current_ = TimeDelta::PlusInfinity();
      idle_hold_rtt_bucket_start_ = at_time;
    }
    idle_hold_rtt_min_current_ = std::min(idle_hold_rtt_min_current_, rtt);
    const TimeDelta baseline =
        std::min(idle_hold_rtt_min_current_, idle_hold_rtt_min_previous_);
    // RustDesk's TestDelay: only delay well above the path itself is queueing.
    const bool bad = rtt - baseline >= kIdleHoldRttExcess &&
                     rtt >= baseline * kIdleHoldRttRatio;
    // A streak broken by a feedback gap is not one episode: a short burst
    // followed by a static screen must not age into "persisting" congestion.
    if (!bad || at_time - idle_hold_last_rtt_sample_ > kIdleHoldRttSampleGap) {
      idle_hold_bad_rtt_samples_ = 0;
      idle_hold_bad_rtt_since_ = Timestamp::PlusInfinity();
    }
    if (bad) {
      if (idle_hold_bad_rtt_samples_++ == 0) idle_hold_bad_rtt_since_ = at_time;
      idle_hold_bad_rtt_last_ = at_time;
    }
    idle_hold_last_rtt_sample_ = at_time;
  }
  if (packets > 0) {
    if (idle_hold_loss_window_start_.IsInfinite()) {
      idle_hold_loss_window_start_ = at_time;
    }
    idle_hold_loss_packets_ += packets;
    idle_hold_lost_packets_ += lost;
    if (at_time - idle_hold_loss_window_start_ >= kIdleHoldCongestionConfirm &&
        idle_hold_loss_packets_ >= kIdleHoldMinLossPackets) {
      const double ratio = static_cast<double>(idle_hold_lost_packets_) /
                           static_cast<double>(idle_hold_loss_packets_);
      idle_hold_bad_loss_windows_ =
          ratio > kIdleHoldLossRatio ? idle_hold_bad_loss_windows_ + 1 : 0;
      idle_hold_loss_ratio_ = ratio;
      idle_hold_loss_window_start_ = at_time;
      idle_hold_loss_packets_ = 0;
      idle_hold_lost_packets_ = 0;
    }
  }
}

std::optional<DataRate> CongestionControl::IdleHoldFloor(
    Timestamp at_time) const {
  if (!idle_rate_hold_enabled_ || !idle_hold_learned_ ||
      idle_hold_rate_ < kIdleHoldMinRate) {
    return std::nullopt;
  }
  if (!idle_hold_in_alr_ && at_time > idle_hold_until_) {
    return std::nullopt;
  }
  return std::min(idle_hold_rate_, max_data_rate_);
}

void CongestionControl::UpdateIdleRateHold(bool in_alr, Timestamp at_time) {
  if (!idle_rate_hold_enabled_) return;
  const DataRate target = bandwidth_estimation_->target_rate();
  const TimeDelta baseline =
      std::min(idle_hold_rtt_min_current_, idle_hold_rtt_min_previous_);
  const TimeDelta rtt = bandwidth_estimation_->round_trip_time();
  // Persistence is measured between the first and the latest bad sample, so
  // it takes fresh feedback, not elapsed process ticks, to confirm.
  const bool rtt_confirmed =
      idle_hold_bad_rtt_samples_ >= kIdleHoldBadRttSamples &&
      idle_hold_bad_rtt_last_ - idle_hold_bad_rtt_since_ >=
          kIdleHoldCongestionConfirm &&
      at_time - idle_hold_bad_rtt_last_ <= kIdleHoldRttSampleGap;
  const bool loss_confirmed =
      idle_hold_bad_loss_windows_ >= kIdleHoldBadLossWindows;
  const bool held = IdleHoldFloor(at_time).has_value();
  // Under load the release is immediate; idle back-off steps are spaced out
  // like RustDesk's ratio cooldown.
  const bool may_act =
      !in_alr || at_time - idle_hold_last_backoff_ >= kIdleHoldBackoffInterval;

  if (held && may_act && (rtt_confirmed || loss_confirmed)) {
    const TimeDelta excess = baseline.IsFinite() && rtt.IsFinite()
                                 ? rtt - baseline
                                 : TimeDelta::Zero();
    DataRate lowered = DataRate::Zero();
    if (!in_alr) {
      // Congestion under load. The estimators may still sit at the floor the
      // hold wrote into them, so push them to what the link delivered (like
      // AIMD's overuse decrease) and re-learn only from a fresh delay-based
      // update; otherwise the release could re-learn the rate it released.
      const std::optional<DataRate> acked =
          acknowledged_bitrate_estimator_->bitrate();
      lowered = acked ? std::min(target, *acked * kProbeDropThroughputFraction)
                      : target;
      LOG_INFO(
          "Idle rate hold released by congestion: held_bps={} new_bps={} "
          "rtt_ms={} rtt_baseline_ms={} loss_window={:.3f}",
          idle_hold_rate_.bps(), lowered.bps(),
          rtt.IsFinite() ? rtt.ms() : -1,
          baseline.IsFinite() ? baseline.ms() : -1, idle_hold_loss_ratio_);
      idle_hold_rate_ = DataRate::Zero();
      idle_hold_learned_ = false;
      idle_hold_until_ = Timestamp::MinusInfinity();
    } else {
      // Congestion while idle (another flow on the link): lower the held
      // rate by RustDesk's confirmed step instead of collapsing to the idle
      // throughput. `target` is the restored floor here, so it is no bound.
      const double factor = std::min(
          0.95, IdleHoldBackoffFactor(
                    excess, loss_confirmed ? idle_hold_loss_ratio_ : 0.0));
      lowered = idle_hold_rate_ * factor;
      LOG_INFO(
          "Idle rate hold lowered by congestion: held_bps={} new_bps={} "
          "rtt_ms={} rtt_baseline_ms={} loss_window={:.3f}",
          idle_hold_rate_.bps(), lowered.bps(), rtt.IsFinite() ? rtt.ms() : -1,
          baseline.IsFinite() ? baseline.ms() : -1, idle_hold_loss_ratio_);
      idle_hold_rate_ = lowered;
    }
    idle_hold_last_backoff_ = at_time;
    idle_hold_bad_rtt_samples_ = 0;
    idle_hold_bad_rtt_since_ = Timestamp::PlusInfinity();
    idle_hold_bad_loss_windows_ = 0;
    idle_hold_in_alr_ = in_alr;
    // The estimators may hold the restored floor; lower them with the hold.
    lowered = std::max(lowered, min_data_rate_);
    if (lowered < target) {
      delay_based_bwe_->SetEstimate(lowered, at_time);
      bandwidth_estimation_->SetSendBitrate(lowered, at_time);
      bandwidth_estimation_->UpdateDelayBasedEstimate(at_time, lowered);
    }
    return;
  }

  if (!idle_hold_learned_) {
    // Nothing measured under load yet (session start, after a reset or a
    // release): do not arm at a configured start rate.
    idle_hold_in_alr_ = in_alr;
    return;
  }
  if (!in_alr) {
    if (idle_hold_in_alr_) {
      // Motion resumed: keep the held floor for a grace window.
      idle_hold_until_ = at_time + kIdleHoldResumeGrace;
      LOG_DEBUG("Idle rate hold: motion resumed held_bps={} target_bps={}",
                idle_hold_rate_.bps(), target.bps());
    }
    if (at_time > idle_hold_until_) {
      // Loaded link, no hold active: the estimators are authoritative.
      idle_hold_rate_ = target;
    } else {
      // Grace window: a lower target is the stale idle estimate; a higher
      // one (probe, increase) is new information.
      idle_hold_rate_ = std::max(idle_hold_rate_, target);
    }
  } else {
    if (!idle_hold_in_alr_) {
      LOG_DEBUG("Idle rate hold: entering idle held_bps={} target_bps={}",
                std::max(idle_hold_rate_, target).bps(), target.bps());
    }
    // Idle never lowers the hold. It may rise only by what the delay-based
    // estimate confirms: AIMD does not increase in ALR, so that is a probe
    // result, whereas the loss-based target drifts up 8%/s without evidence.
    idle_hold_rate_ = std::max(
        idle_hold_rate_, std::min(target, delay_based_bwe_->last_estimate()));
  }
  idle_hold_in_alr_ = in_alr;
}

void CongestionControl::ApplyIdleRateHold(Timestamp at_time) {
  const auto floor = IdleHoldFloor(at_time);
  if (!floor) return;
  const DataRate target = bandwidth_estimation_->target_rate();
  if (target >= *floor) return;
  // Rate-limit the log: idle traffic may hit this on every feedback.
  if (at_time - idle_hold_last_restore_log_ >= TimeDelta::Seconds(5)) {
    idle_hold_last_restore_log_ = at_time;
    LOG_INFO(
        "Idle rate hold restored target: from_bps={} to_bps={} in_alr={}",
        target.bps(), floor->bps(), idle_hold_in_alr_);
  }
  // Restore the held rate in the estimators themselves so the pacer, encoder
  // target and ALR budget all agree; a delay-based limit learned on an idle
  // link would otherwise cap the restored target again. SetSendBitrate()
  // clears the delay-based cap, so re-apply it at the floor: without it the
  // loss-based estimate would climb 8%/s to the configured maximum.
  delay_based_bwe_->SetEstimate(*floor, at_time);
  bandwidth_estimation_->SetSendBitrate(*floor, at_time);
  bandwidth_estimation_->UpdateDelayBasedEstimate(at_time, *floor);
}

NetworkControlUpdate CongestionControl::SetRelayPath(bool relay_path,
                                                     Timestamp at_time) {
  NetworkControlUpdate update;
  if (relay_path_ == relay_path) {
    return update;
  }

  pending_probe_.reset();
  relay_path_ = relay_path;
  probe_controller_->SetRelayPath(relay_path);
  // A new path invalidates the held rate and the RTT baseline. A relay move
  // deliberately lowers the target, which the hold must not undo.
  ResetIdleRateHold();
  const DataRate desired_starting_rate =
      relay_path_ ? kRelayStartingRate : kDirectStartingRate;

  if (initial_config_) {
    initial_config_->constraints.starting_rate = desired_starting_rate;
    LOG_INFO("Congestion-control startup profile updated: path={} "
             "starting_bitrate_bps={}",
             relay_path_ ? "relay" : "direct",
             desired_starting_rate.bps());
    return update;
  }

  // Lower an established controller immediately when ICE moves onto a relay.
  // On a new direct path, measure the available headroom before raising media.
  if (!relay_path_ && network_available_) {
    update.probe_cluster_configs =
        probe_controller_->RequestProbeOnPathChange(at_time);
    LOG_INFO("Congestion-control probe profile updated: path=direct "
             "starting_bitrate_unchanged_bps={}",
             bandwidth_estimation_->target_rate().bps());
    return update;
  }

  TargetRateConstraints constraints;
  constraints.at_time = at_time;
  constraints.min_data_rate = min_data_rate_;
  constraints.max_data_rate = max_data_rate_;
  constraints.starting_rate = desired_starting_rate;
  update.probe_cluster_configs = ResetConstraints(constraints);
  MaybeTriggerOnNetworkChanged(&update, at_time);
  LOG_INFO("Congestion-control startup profile applied: path={} "
           "starting_bitrate_bps={}",
           relay_path_ ? "relay" : "direct",
           desired_starting_rate.bps());
  return update;
}

NetworkControlUpdate CongestionControl::OnNetworkAvailability(
    NetworkAvailability msg) {
  network_available_ = msg.network_available;
  if (!network_available_) {
    pending_probe_.reset();
    // The link may come back as a different path; do not hold its old rate.
    ResetIdleRateHold();
  }
  NetworkControlUpdate update;
  update.probe_cluster_configs = probe_controller_->OnNetworkAvailability(msg);
  return update;
}

NetworkControlUpdate CongestionControl::OnProcessInterval(ProcessInterval msg) {
  NetworkControlUpdate update;
  probe_bitrate_estimator_->RemoveExpiredClusters(msg.at_time);
  if (initial_config_) {
    update.probe_cluster_configs =
        ResetConstraints(initial_config_->constraints);
    update.pacer_config = GetPacingRates(msg.at_time);

    if (initial_config_->stream_based_config.requests_alr_probing) {
      probe_controller_->EnablePeriodicAlrProbing(
          *initial_config_->stream_based_config.requests_alr_probing);
    }
    if (initial_config_->stream_based_config.enable_repeated_initial_probing) {
      probe_controller_->EnableRepeatedInitialProbing(
          *initial_config_->stream_based_config
               .enable_repeated_initial_probing);
    }
    std::optional<DataRate> total_bitrate =
        initial_config_->stream_based_config.max_total_allocated_bitrate;
    if (total_bitrate) {
      auto probes = probe_controller_->OnMaxTotalAllocatedBitrate(
          *total_bitrate, msg.at_time);
      update.probe_cluster_configs.insert(update.probe_cluster_configs.end(),
                                          probes.begin(), probes.end());
    }
    initial_config_.reset();
  }

  bandwidth_estimation_->UpdateEstimate(msg.at_time);
  std::optional<int64_t> start_time_ms =
      alr_detector_->GetApplicationLimitedRegionStartTime();
  probe_controller_->SetAlrStartTimeMs(start_time_ms);

  auto probes = probe_controller_->Process(msg.at_time);
  update.probe_cluster_configs.insert(update.probe_cluster_configs.end(),
                                      probes.begin(), probes.end());
  update.congestion_window = current_data_window_;

  MaybeTriggerOnNetworkChanged(&update, msg.at_time);
  MaybeApplyPendingProbe(&update, msg.at_time);
  return update;
}

bool CongestionControl::ProbeNetworkHealthy() const {
  const TimeDelta rtt = bandwidth_estimation_->round_trip_time();
  return network_available_ &&
         delay_based_bwe_->last_state() != BandwidthUsage::kBwOverusing &&
         bandwidth_estimation_->fraction_loss() / 255.0 < 0.05 &&
         rtt.IsFinite() && rtt < TimeDelta::Millis(300);
}

std::optional<DataRate> CongestionControl::FilterProbeResult(
    DataRate bitrate, int probe_id, Timestamp at_time) {
  if (pending_probe_ && pending_probe_->id != probe_id) {
    pending_probe_.reset();
  }
  if (!ProbeNetworkHealthy()) {
    pending_probe_.reset();
    return bitrate;
  }
  const DataRate reference =
      pending_probe_ ? pending_probe_->reference : last_loss_based_target_rate_;
  // Compare against the capped media target, not a raw delay estimate that
  // may exceed the configured maximum. Small revisions remain immediate.
  if (probe_id <= last_settled_probe_id_ || reference <= DataRate::Zero() ||
      bitrate >= reference * 0.90) {
    pending_probe_.reset();
    return bitrate;
  }
  if (pending_probe_) {
    // Revisions update the candidate, never the original deadline.
    pending_probe_->bitrate = bitrate;
  } else {
    pending_probe_ = PendingProbe{probe_id, bitrate, reference,
                                  at_time + kProbeEstimateSettleTime};
  }
  return std::nullopt;
}

void CongestionControl::MaybeApplyPendingProbe(NetworkControlUpdate* update,
                                               Timestamp at_time) {
  if (!pending_probe_) return;
  if (!ProbeNetworkHealthy()) {
    pending_probe_.reset();
    return;
  }
  if (at_time < pending_probe_->deadline) return;
  if (at_time - pending_probe_->deadline > TimeDelta::Seconds(1)) {
    pending_probe_.reset();
    return;
  }
  const PendingProbe probe = *pending_probe_;
  pending_probe_.reset();
  last_settled_probe_id_ = probe.id;
  // Ordinary delay/loss feedback continues throughout the settling window.
  // Do not use an old candidate to undo a newer congestion-driven reduction.
  if (bandwidth_estimation_->target_rate() < probe.bitrate) {
    return;
  }
  const auto result = delay_based_bwe_->ApplyProbeResult(
      probe.bitrate, at_time, acknowledged_bitrate_estimator_->bitrate(),
      alr_detector_->GetApplicationLimitedRegionStartTime().has_value());
  if (!result.updated || !result.probe) return;
  bandwidth_estimation_->SetSendBitrate(result.target_bitrate, at_time);
  bandwidth_estimation_->UpdateDelayBasedEstimate(at_time,
                                                  result.target_bitrate);
  MaybeTriggerOnNetworkChanged(update, at_time);
}

void CongestionControl::ClampConstraints() {
  // TODO(holmer): We should make sure the default bitrates are set to 10 kbps,
  // and that we don't try to set the min bitrate to 0 from any applications.
  // The congestion controller should allow a min bitrate of 0.
  min_data_rate_ = std::max(min_target_rate_, kCongestionControllerMinBitrate);
  if (use_min_allocatable_as_lower_bound_) {
    min_data_rate_ = std::max(min_data_rate_, min_total_allocated_bitrate_);
  }
  if (max_data_rate_ < min_data_rate_) {
    LOG_WARN("max bitrate smaller than min bitrate");
    max_data_rate_ = min_data_rate_;
  }
  if (starting_rate_ && starting_rate_ < min_data_rate_) {
    LOG_WARN("start bitrate smaller than min bitrate");
    starting_rate_ = min_data_rate_;
  }
}

std::vector<ProbeClusterConfig> CongestionControl::ResetConstraints(
    TargetRateConstraints new_constraints) {
  pending_probe_.reset();
  min_target_rate_ = new_constraints.min_data_rate.value_or(DataRate::Zero());
  max_data_rate_ =
      new_constraints.max_data_rate.value_or(DataRate::PlusInfinity());
  starting_rate_ = new_constraints.starting_rate;
  ClampConstraints();

  bandwidth_estimation_->SetBitrates(starting_rate_, min_data_rate_,
                                     max_data_rate_, new_constraints.at_time);

  if (starting_rate_) delay_based_bwe_->SetStartBitrate(*starting_rate_);
  delay_based_bwe_->SetMinBitrate(min_data_rate_);

  return probe_controller_->SetBitrates(
      min_data_rate_, starting_rate_.value_or(DataRate::Zero()), max_data_rate_,
      new_constraints.at_time);
}

NetworkControlUpdate CongestionControl::OnTransportLossReport(
    TransportLossReport msg) {
  if (packet_feedback_only_) {
    return NetworkControlUpdate();
  }
  int64_t total_packets_delta =
      msg.packets_received_delta + msg.packets_lost_delta;
  bandwidth_estimation_->UpdatePacketsLost(
      msg.packets_lost_delta, total_packets_delta, msg.receive_time);
  return NetworkControlUpdate();
}

void CongestionControl::OnSentPacket(size_t bytes_sent, Timestamp at_time) {
  alr_detector_->OnBytesSent(bytes_sent, at_time.ms());
  std::optional<int64_t> alr_start_time =
      alr_detector_->GetApplicationLimitedRegionStartTime();
  acknowledged_bitrate_estimator_->SetAlr(alr_start_time.has_value());
}

NetworkControlUpdate CongestionControl::OnTransportPacketsFeedback(
    TransportPacketsFeedback report) {
  if (report.packet_feedbacks.empty()) {
    // TODO(bugs.webrtc.org/10125): Design a better mechanism to safe-guard
    // against building very large network queues.
    return NetworkControlUpdate();
  }

  // if (congestion_window_pushback_controller_) {
  //   congestion_window_pushback_controller_->UpdateOutstandingData(
  //       report.data_in_flight.bytes());
  // }
  TimeDelta max_feedback_rtt = TimeDelta::MinusInfinity();
  TimeDelta min_propagation_rtt = TimeDelta::PlusInfinity();
  Timestamp max_recv_time = Timestamp::MinusInfinity();

  std::vector<PacketResult> feedbacks = report.ReceivedWithSendInfo();
  for (const auto& feedback : feedbacks)
    max_recv_time = std::max(max_recv_time, feedback.receive_time);

  for (const auto& feedback : feedbacks) {
    TimeDelta feedback_rtt =
        report.feedback_time - feedback.sent_packet.send_time;
    TimeDelta min_pending_time = max_recv_time - feedback.receive_time;
    TimeDelta propagation_rtt = feedback_rtt - min_pending_time;
    max_feedback_rtt = std::max(max_feedback_rtt, feedback_rtt);
    min_propagation_rtt = std::min(min_propagation_rtt, propagation_rtt);
  }

  if (max_feedback_rtt.IsFinite()) {
    feedback_max_rtts_.push_back(max_feedback_rtt.ms());
    const size_t kMaxFeedbackRttWindow = 32;
    if (feedback_max_rtts_.size() > kMaxFeedbackRttWindow)
      feedback_max_rtts_.pop_front();
    // TODO(srte): Use time since last unacknowledged packet.
    bandwidth_estimation_->UpdatePropagationRtt(report.feedback_time,
                                                min_propagation_rtt);
  }
  if (packet_feedback_only_) {
    if (!feedback_max_rtts_.empty()) {
      int64_t sum_rtt_ms =
          std::accumulate(feedback_max_rtts_.begin(), feedback_max_rtts_.end(),
                          static_cast<int64_t>(0));
      int64_t mean_rtt_ms = sum_rtt_ms / feedback_max_rtts_.size();
      if (delay_based_bwe_) {
        delay_based_bwe_->OnRttUpdate(TimeDelta::Millis(mean_rtt_ms));
      }
    }

    TimeDelta feedback_min_rtt = TimeDelta::PlusInfinity();
    for (const auto& packet_feedback : feedbacks) {
      TimeDelta pending_time = max_recv_time - packet_feedback.receive_time;
      TimeDelta rtt = report.feedback_time -
                      packet_feedback.sent_packet.send_time - pending_time;
      // Value used for predicting NACK round trip time in FEC controller.
      feedback_min_rtt = std::min(rtt, feedback_min_rtt);
    }
    if (feedback_min_rtt.IsFinite()) {
      bandwidth_estimation_->UpdateRtt(feedback_min_rtt, report.feedback_time);
    }

    const std::vector<PacketResult> with_feedback = report.PacketsWithFeedback();
    int64_t lost_in_report = 0;
    for (const auto& packet_feedback : with_feedback) {
      if (!packet_feedback.IsReceived()) ++lost_in_report;
    }
    expected_packets_since_last_loss_update_ += with_feedback.size();
    lost_packets_since_last_loss_update_ += lost_in_report;
    UpdateIdleHoldFeedback(feedback_min_rtt,
                           static_cast<int64_t>(with_feedback.size()),
                           lost_in_report, /*in_alr=*/true,
                           /*delay_updated=*/false, report.feedback_time);
    if (report.feedback_time > next_loss_update_) {
      next_loss_update_ =
          report.feedback_time + TimeDelta::Millis(kLossUpdateInterval);
      bandwidth_estimation_->UpdatePacketsLost(
          lost_packets_since_last_loss_update_,
          expected_packets_since_last_loss_update_, report.feedback_time);
      expected_packets_since_last_loss_update_ = 0;
      lost_packets_since_last_loss_update_ = 0;
    }
  }
  std::optional<int64_t> alr_start_time =
      alr_detector_->GetApplicationLimitedRegionStartTime();

  if (previously_in_alr_ && !alr_start_time.has_value()) {
    int64_t now_ms = report.feedback_time.ms();
    acknowledged_bitrate_estimator_->SetAlrEndedTime(report.feedback_time);
    probe_controller_->SetAlrEndedTimeMs(now_ms);
  }
  previously_in_alr_ = alr_start_time.has_value();

  acknowledged_bitrate_estimator_->IncomingPacketFeedbackVector(
      report.SortedByReceiveTime());
  auto acknowledged_bitrate = acknowledged_bitrate_estimator_->bitrate();
  bandwidth_estimation_->SetAcknowledgedRate(acknowledged_bitrate,
                                             report.feedback_time);
  for (const auto& feedback : report.SortedByReceiveTime()) {
    if (feedback.sent_packet.pacing_info.probe_cluster_id !=
        PacedPacketInfo::kNotAProbe) {
      auto probe_result =
          probe_bitrate_estimator_->HandleProbeAndEstimateBitrate(feedback);
      if (probe_result) {
        probe_controller_->OnProbeResult(
            feedback.sent_packet.pacing_info.probe_cluster_id,
            report.feedback_time);
      }
    }
  }

  int probe_id = -1;
  std::optional<DataRate> probe_bitrate =
      probe_bitrate_estimator_->FetchAndResetLastEstimatedBitrate(&probe_id);
  if (limit_probes_lower_than_throughput_estimate_ && probe_bitrate &&
      acknowledged_bitrate) {
    // Limit the backoff to something slightly below the acknowledged
    // bitrate. ("Slightly below" because we want to drain the queues
    // if we are actually overusing.)
    // The acknowledged bitrate shouldn't normally be higher than the delay
    // based estimate, but it could happen e.g. due to packet bursts or
    // encoder overshoot. We use std::min to ensure that a probe result
    // below the current BWE never causes an increase.
    DataRate limit =
        std::min(delay_based_bwe_->last_estimate(),
                 *acknowledged_bitrate * kProbeDropThroughputFraction);
    probe_bitrate = std::max(*probe_bitrate, limit);
  }

  if (probe_bitrate) {
    probe_bitrate =
        FilterProbeResult(*probe_bitrate, probe_id, report.feedback_time);
  }

  NetworkControlUpdate update;
  bool recovered_from_overuse = false;

  DelayBasedBwe::Result result;
  result = delay_based_bwe_->IncomingPacketFeedbackVector(
      report, acknowledged_bitrate, probe_bitrate, alr_start_time.has_value());

  if (result.updated) {
    if (result.probe) {
      bandwidth_estimation_->SetSendBitrate(result.target_bitrate,
                                            report.feedback_time);
    }
    // Since SetSendBitrate now resets the delay-based estimate, we have to
    // call UpdateDelayBasedEstimate after SetSendBitrate.
    bandwidth_estimation_->UpdateDelayBasedEstimate(report.feedback_time,
                                                    result.target_bitrate);
  }
  bandwidth_estimation_->UpdateLossBasedEstimator(
      report, result.delay_detector_state, probe_bitrate,
      alr_start_time.has_value());
  if (!ProbeNetworkHealthy()) pending_probe_.reset();
  if (result.updated) {
    // A delay-based update from feedback under load is what lets the idle
    // hold learn a rate (not a start rate or a rate it has just restored).
    UpdateIdleHoldFeedback(TimeDelta::PlusInfinity(), 0, 0,
                           alr_start_time.has_value(), /*delay_updated=*/true,
                           report.feedback_time);
    // Update the estimate in the ProbeController, in case we want to probe.
    MaybeTriggerOnNetworkChanged(&update, report.feedback_time);
  }

  recovered_from_overuse = result.recovered_from_overuse;
  MaybeApplyPendingProbe(&update, report.feedback_time);

  if (recovered_from_overuse) {
    probe_controller_->SetAlrStartTimeMs(alr_start_time);
    auto probes = probe_controller_->RequestProbe(report.feedback_time);
    update.probe_cluster_configs.insert(update.probe_cluster_configs.end(),
                                        probes.begin(), probes.end());
  }

  // No valid RTT could be because send-side BWE isn't used, in which case
  // we don't try to limit the outstanding packets.
  // if (congestion_window_pushback_controller_ && current_data_window_) {
  //   congestion_window_pushback_controller_->SetDataWindow(
  //       *current_data_window_);
  // } else {
  //   update.congestion_window = current_data_window_;
  // }

  return update;
}

void CongestionControl::MaybeTriggerOnNetworkChanged(
    NetworkControlUpdate* update, Timestamp at_time) {
  // Every estimator path (feedback, loss reports, probes, process ticks)
  // publishes through here. Keep an idle desktop from eroding the target
  // before it is reported to the encoder, the pacer and the ALR budget.
  if (idle_rate_hold_enabled_) {
    UpdateIdleRateHold(
        alr_detector_->GetApplicationLimitedRegionStartTime().has_value(),
        at_time);
    ApplyIdleRateHold(at_time);
  }
  uint8_t fraction_loss = bandwidth_estimation_->fraction_loss();
  TimeDelta round_trip_time = bandwidth_estimation_->round_trip_time();
  DataRate loss_based_target_rate = bandwidth_estimation_->target_rate();
  LossBasedState loss_based_state = bandwidth_estimation_->loss_based_state();
  DataRate pushback_target_rate = loss_based_target_rate;

  double cwnd_reduce_ratio = 0.0;
  // if (congestion_window_pushback_controller_) {
  //   int64_t pushback_rate =
  //       congestion_window_pushback_controller_->UpdateTargetBitrate(
  //           loss_based_target_rate.bps());
  //   pushback_rate = std::max<int64_t>(bandwidth_estimation_->GetMinBitrate(),
  //                                     pushback_rate);
  //   pushback_target_rate = DataRate::BitsPerSec(pushback_rate);
  //   if (rate_control_settings_.UseCongestionWindowDropFrameOnly()) {
  //     cwnd_reduce_ratio = static_cast<double>(loss_based_target_rate.bps() -
  //                                             pushback_target_rate.bps()) /
  //                         loss_based_target_rate.bps();
  //   }
  // }
  DataRate stable_target_rate =
      bandwidth_estimation_->GetEstimatedLinkCapacity();
  stable_target_rate = std::min(stable_target_rate, pushback_target_rate);

  const bool in_alr =
      alr_detector_->GetApplicationLimitedRegionStartTime().has_value();

  if ((loss_based_target_rate != last_loss_based_target_rate_) ||
      in_alr != last_reported_in_alr_ ||
      (loss_based_state != last_loss_base_state_) ||
      (fraction_loss != last_estimated_fraction_loss_) ||
      (round_trip_time != last_estimated_round_trip_time_) ||
      (pushback_target_rate != last_pushback_target_rate_) ||
      (stable_target_rate != last_stable_target_rate_)) {
    last_loss_based_target_rate_ = loss_based_target_rate;
    last_pushback_target_rate_ = pushback_target_rate;
    last_estimated_fraction_loss_ = fraction_loss;
    last_estimated_round_trip_time_ = round_trip_time;
    last_stable_target_rate_ = stable_target_rate;
    last_loss_base_state_ = loss_based_state;
    last_reported_in_alr_ = in_alr;

    alr_detector_->SetEstimatedBitrate(loss_based_target_rate.bps());

    TimeDelta bwe_period = delay_based_bwe_->GetExpectedBwePeriod();

    TargetTransferRate target_rate_msg;
    target_rate_msg.at_time = at_time;
    // if (rate_control_settings_.UseCongestionWindowDropFrameOnly()) {
    //   target_rate_msg.target_rate = loss_based_target_rate;
    //   target_rate_msg.cwnd_reduce_ratio = cwnd_reduce_ratio;
    // } else
    {
      target_rate_msg.target_rate = pushback_target_rate;
    }
    target_rate_msg.stable_target_rate = stable_target_rate;
    target_rate_msg.network_estimate.at_time = at_time;
    target_rate_msg.network_estimate.round_trip_time = round_trip_time;
    target_rate_msg.network_estimate.loss_rate_ratio = fraction_loss / 255.0f;
    target_rate_msg.network_estimate.bwe_period = bwe_period;
    target_rate_msg.network_estimate.in_alr = in_alr;

    update->target_rate = target_rate_msg;

    auto probes = probe_controller_->SetEstimatedBitrate(
        loss_based_target_rate,
        GetBandwidthLimitedCause(bandwidth_estimation_->loss_based_state(),
                                 bandwidth_estimation_->IsRttAboveLimit(),
                                 delay_based_bwe_->last_state()),
        at_time,
        fraction_loss / 255.0 < 0.05 && round_trip_time.IsFinite() &&
            round_trip_time < TimeDelta::Millis(300));
    update->probe_cluster_configs.insert(update->probe_cluster_configs.end(),
                                         probes.begin(), probes.end());
    update->pacer_config = GetPacingRates(at_time);
    // LOG_INFO("bwe {} pushback_target_bps={} estimate_bps={}",
    // at_time.ms(),
    //          last_pushback_target_rate_.bps(),
    //          loss_based_target_rate.bps());
  }
}

PacerConfig CongestionControl::GetPacingRates(Timestamp at_time) const {
  // Pacing rate is based on target rate before congestion window pushback,
  // because we don't want to build queues in the pacer when pushback occurs.
  DataRate pacing_rate = DataRate::Zero();
  if (pace_at_max_of_bwe_and_lower_link_capacity_ && estimate_ &&
      !bandwidth_estimation_->PaceAtLossBasedEstimate()) {
    pacing_rate =
        std::max({min_total_allocated_bitrate_, estimate_->link_capacity_lower,
                  last_loss_based_target_rate_}) *
        pacing_factor_;
  } else {
    pacing_rate =
        std::max(min_total_allocated_bitrate_, last_loss_based_target_rate_) *
        pacing_factor_;
  }
  // While the idle hold protects a rate, never pace media below it: otherwise
  // the first frames after motion resumes queue up and admission drops them.
  if (const auto floor = IdleHoldFloor(at_time)) {
    pacing_rate = std::max(pacing_rate, *floor * pacing_factor_);
  }
  if (limit_pacingfactor_by_upper_link_capacity_estimate_ && estimate_ &&
      estimate_->link_capacity_upper.IsFinite() &&
      pacing_rate > estimate_->link_capacity_upper) {
    pacing_rate =
        std::max({estimate_->link_capacity_upper, min_total_allocated_bitrate_,
                  last_loss_based_target_rate_});
  }

  DataRate padding_rate =
      (last_loss_base_state_ == LossBasedState::kIncreaseUsingPadding)
          ? std::max(max_padding_rate_, last_loss_based_target_rate_)
          : max_padding_rate_;
  padding_rate = std::min(padding_rate, last_pushback_target_rate_);
  PacerConfig msg;
  msg.at_time = at_time;
  msg.time_window = TimeDelta::Seconds(1);
  msg.data_window = pacing_rate * msg.time_window;
  msg.pad_window = padding_rate * msg.time_window;
  return msg;
}
}  // namespace minirtc
