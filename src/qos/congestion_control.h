#ifndef _CONGESTION_CONTROL_H_
#define _CONGESTION_CONTROL_H_

#include <cstddef>
#include <deque>
#include <memory>

#include "acknowledged_bitrate_estimator.h"
#include "acknowledged_bitrate_estimator_interface.h"
#include "alr_detector.h"
#include "api/network_state_predictor.h"
#include "api/transport/network_control.h"
#include "api/transport/network_types.h"
#include "congestion_window_pushback_controller.h"
#include "delay_based_bwe.h"
#include "probe_controller.h"
#include "send_side_bandwidth_estimation.h"

namespace minirtc {

using namespace webrtc;

class CongestionControl {
 public:
  CongestionControl();
  ~CongestionControl();

 public:
  NetworkControlUpdate OnNetworkAvailability(NetworkAvailability msg);

  NetworkControlUpdate OnProcessInterval(ProcessInterval msg);

  NetworkControlUpdate OnTransportLossReport(TransportLossReport msg);

  NetworkControlUpdate OnTransportPacketsFeedback(
      TransportPacketsFeedback report);

  void OnSentPacket(size_t bytes_sent, Timestamp at_time);

  void SetRepeatedInitialProbing(bool enable);

  NetworkControlUpdate SetRelayPath(bool relay_path, Timestamp at_time);

  // Screen content: an application-limited (idle) desktop is not evidence of
  // a smaller link. Hold the last loaded-link rate while idle and briefly
  // after motion resumes; only congestion confirmed under load releases it,
  // so motion resumes at the pre-idle bitrate.
  void SetIdleRateHold(bool enabled);

  void MaybeTriggerOnNetworkChanged(NetworkControlUpdate* update,
                                    Timestamp at_time);

 private:
  void ClampConstraints();
  std::vector<ProbeClusterConfig> ResetConstraints(
      TargetRateConstraints new_constraints);
  PacerConfig GetPacingRates(Timestamp at_time) const;
  bool ProbeNetworkHealthy() const;
  std::optional<DataRate> FilterProbeResult(DataRate bitrate, int probe_id,
                                            Timestamp at_time);
  void MaybeApplyPendingProbe(NetworkControlUpdate* update, Timestamp at_time);

  // Returns the rate the idle hold currently protects, or nullopt when the
  // hold is disabled, not armed or released by confirmed congestion.
  std::optional<DataRate> IdleHoldFloor(Timestamp at_time) const;
  void ResetIdleRateHold();
  // Per-feedback congestion evidence: the RTT baseline, consecutive high-RTT
  // samples and loss windows. Counted in and out of ALR (the idle step-down
  // relies on it). `delay_updated` marks a delay-based update under load,
  // the only evidence that lets the hold learn a rate.
  void UpdateIdleHoldFeedback(TimeDelta rtt, int64_t packets, int64_t lost,
                              bool in_alr, bool delay_updated,
                              Timestamp at_time);
  void UpdateIdleRateHold(bool in_alr, Timestamp at_time);
  void ApplyIdleRateHold(Timestamp at_time);

 private:
  const bool packet_feedback_only_;
  const bool use_min_allocatable_as_lower_bound_;
  const bool ignore_probes_lower_than_network_estimate_;
  const bool limit_probes_lower_than_throughput_estimate_;
  const bool pace_at_max_of_bwe_and_lower_link_capacity_;
  const bool limit_pacingfactor_by_upper_link_capacity_estimate_;

  const std::unique_ptr<ProbeController> probe_controller_;
  const std::unique_ptr<CongestionWindowPushbackController>
      congestion_window_pushback_controller_;

  std::unique_ptr<SendSideBandwidthEstimation> bandwidth_estimation_;
  std::unique_ptr<AlrDetector> alr_detector_;
  std::unique_ptr<ProbeBitrateEstimator> probe_bitrate_estimator_;
  std::unique_ptr<DelayBasedBwe> delay_based_bwe_;
  std::unique_ptr<AcknowledgedBitrateEstimatorInterface>
      acknowledged_bitrate_estimator_;

  std::optional<NetworkControllerConfig> initial_config_;

  DataRate min_target_rate_ = DataRate::Zero();
  DataRate min_data_rate_ = DataRate::Zero();
  DataRate max_data_rate_ = DataRate::PlusInfinity();
  std::optional<DataRate> starting_rate_;

  bool first_packet_sent_ = false;

  std::optional<NetworkStateEstimate> estimate_;

  Timestamp next_loss_update_ = Timestamp::MinusInfinity();
  int lost_packets_since_last_loss_update_ = 0;
  int expected_packets_since_last_loss_update_ = 0;

  std::deque<int64_t> feedback_max_rtts_;

  DataRate last_loss_based_target_rate_;
  DataRate last_pushback_target_rate_ = DataRate::Zero();
  DataRate last_stable_target_rate_;
  LossBasedState last_loss_base_state_;

  std::optional<uint8_t> last_estimated_fraction_loss_ = 0;
  TimeDelta last_estimated_round_trip_time_ = TimeDelta::PlusInfinity();

  double pacing_factor_;
  DataRate min_total_allocated_bitrate_;
  DataRate max_padding_rate_;

  bool previously_in_alr_ = false;
  bool last_reported_in_alr_ = false;
  bool network_available_ = false;
  bool relay_path_ = false;

  struct PendingProbe {
    int id;
    DataRate bitrate;
    DataRate reference;
    Timestamp deadline;
  };
  std::optional<PendingProbe> pending_probe_;
  int last_settled_probe_id_ = -1;

  bool idle_rate_hold_enabled_ = false;
  // Last target published under load and outside the post-idle grace
  // window. Only learned once idle_hold_learned_ is set.
  DataRate idle_hold_rate_ = DataRate::Zero();
  // Set by a delay-based update from feedback outside ALR; cleared on reset
  // and on release, so the hold never arms at a start rate or at the rate it
  // has just released.
  bool idle_hold_learned_ = false;
  // The hold protects idle periods plus a grace window after motion resumes,
  // so the first large frames can drain at the held rate.
  Timestamp idle_hold_until_ = Timestamp::MinusInfinity();
  bool idle_hold_in_alr_ = false;
  // RTT baseline: minimum of the current and the previous bucket.
  TimeDelta idle_hold_rtt_min_current_ = TimeDelta::PlusInfinity();
  TimeDelta idle_hold_rtt_min_previous_ = TimeDelta::PlusInfinity();
  Timestamp idle_hold_rtt_bucket_start_ = Timestamp::MinusInfinity();
  // Consecutive feedback RTT samples far above the baseline: first and last.
  int idle_hold_bad_rtt_samples_ = 0;
  Timestamp idle_hold_bad_rtt_since_ = Timestamp::PlusInfinity();
  Timestamp idle_hold_bad_rtt_last_ = Timestamp::MinusInfinity();
  Timestamp idle_hold_last_rtt_sample_ = Timestamp::MinusInfinity();
  // Loss measured over short windows; consecutive windows above the limit.
  Timestamp idle_hold_loss_window_start_ = Timestamp::MinusInfinity();
  int64_t idle_hold_loss_packets_ = 0;
  int64_t idle_hold_lost_packets_ = 0;
  int idle_hold_bad_loss_windows_ = 0;
  double idle_hold_loss_ratio_ = 0.0;
  // Last confirmed-congestion action; idle back-off steps are spaced out.
  Timestamp idle_hold_last_backoff_ = Timestamp::MinusInfinity();
  Timestamp idle_hold_last_restore_log_ = Timestamp::MinusInfinity();

  std::optional<DataSize> current_data_window_;
};
}  // namespace minirtc

#endif
