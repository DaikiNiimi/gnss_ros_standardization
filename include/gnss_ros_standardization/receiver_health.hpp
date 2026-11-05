#pragma once
#include <chrono>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <iomanip>
#include <cstring>
#include "gnss_ros_standardization/binary_framer.hpp"
#include "gnss_ros_standardization/protocol_id_catalog.hpp"
#include "gnss_ros_standardization/receiver_diagnostics.h"
#include <stdexcept>
#include <string>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace gnss_ros_standardization::wire {
// Counts stages independently: an integrity-checked frame is not a decoded
// observation, and decoder return 0 may mean a partial epoch or auxiliary data.
// This monitor never infers unsupported-signal counts from that return value.
class ReceiverHealth {
 public:
  using Clock=std::chrono::steady_clock;
  explicit ReceiverHealth(Clock::time_point start=Clock::now()) : last_report_(start) {}
  void configure(double period) {
    if(!std::isfinite(period) || period<1 || period>60)
      throw std::invalid_argument("diagnostics.period_s must be in [1, 60]");
    period_=period;
  }
  void input(int bytes,Clock::time_point now=Clock::now()) {
    if(bytes>0) { bytes_+=bytes; last_input_=now; have_input_=true; }
  }
  void frame(unsigned id,unsigned revision=0) { ++frames_; last_id_=id; last_revision_=revision; }
  // Inspect metadata only; this does not accept, reject or mutate decoder input.
  void inspect(Protocol protocol,const Frame& f,bool rawx_data0=false) {
    const unsigned id=protocol==Protocol::Ubx ? (unsigned(f.cls)<<8)|f.id : f.id;
    const bool known=protocol==Protocol::Ubx ? knownUBXId(id) :
                     protocol==Protocol::Sbf ? knownSBFId(id) : knownOEM4Id(id);
    if(!known) { ++unknown_ids_; return; }
    const bool no_decoder=protocol==Protocol::Ubx ? noDecoderUBXId(id) :
                          protocol==Protocol::Sbf ? noDecoderSBFId(id) : noDecoderOEM4Id(id);
    if(no_decoder) ++known_without_decoder_;
    bool checked=false;
    if(protocol==Protocol::Ubx && id==0x0215 && f.payload.size()>=16) {
      checked=true; last_payload_version_=rawx_data0 ? -1 : f.payload[13];
      if(!rawx_data0 && f.payload[13]>1) ++unknown_versions_;
      // Little endian IEEE754 bytes, independent of host endianness.
      uint64_t bits=0; for(unsigned i=0;i<8;++i) bits|=uint64_t(f.payload[i])<<(8*i);
      double tow; std::memcpy(&tow,&bits,sizeof(tow));
      if(!u16(f.payload.data()+8) || !std::isfinite(tow) || tow<0 || tow>=604800) timeUnresolved();
    } else if(protocol==Protocol::Ubx && id==0x0210 && f.payload.size()>=8) {
      // Legacy RAW has no payload-version byte.
      checked=true;
      if(!u16(f.payload.data()+4) || u32(f.payload.data())>=604800000) timeUnresolved();
    } else if(protocol==Protocol::Sbf) {
      if(id==4027 || id==4004) { checked=true; if(f.revision>1) ++unknown_versions_; }
      if(f.payload.size()>=6 && (u16(f.payload.data()+4)==65535 || u32(f.payload.data())>=604800000)) timeUnresolved();
    }
    if(!checked) ++version_unchecked_;
  }
  void inspectRtcm(unsigned id) {
    if(!knownRTCM3Id(id)) ++unknown_ids_;
    else { ++version_unchecked_; if(noDecoderRTCM3Id(id)) ++known_without_decoder_; }
  }
  void framing(const Framer& f) {
    rejected_=f.rejected(); checksum_rejected_=f.checksumRejected();
    malformed_=f.malformed(); incomplete_expired_=f.expired();
  }
  void timeUnresolved() { ++time_unresolved_; }
  void timeWithheld() { ++time_withheld_; }
  ReceiverDiagnosticScope backendScope() { backend_observed_=true; return ReceiverDiagnosticScope(backend_); }
  void pvtPending(bool pending) { pvt_pending_=pending; pvt_observed_=true; }
  void pvtIncompleteDiscarded() { ++pvt_incomplete_epochs_; }
  void imuPending(bool pending) { imu_pending_=pending; imu_observed_=true; }
  void assemblyPending(bool pending) {
    if(pending && !assembly_pending_) ++assembly_started_;
    assembly_pending_=pending;
  }
  void observationCapacity(size_t satellites,size_t limit) { if(satellites>=limit) ++capacity_at_limit_; }
  void backendResult(int result) {
    if(result<0) ++payload_errors_;
    else if(result==1) ++decoded_epochs_;
  }
  void rejected(uint64_t total) { rejected_=total; }
  void rejectFrame() { ++rejected_; ++checksum_rejected_; }
  void unsupportedNavigation() { ++unsupported_navigation_; }
  void snapshotRejected(size_t count) { snapshot_rejected_=count; }
  void limits(unsigned observations,unsigned primary,unsigned extra,unsigned bds) {
    maxobs_=observations; nfreq_=primary; nexobs_=extra; bds_prn_=bds;
  }
  void msmSplitDetected() { ++msm_split_detected_; }
  void msmTimeTieWithheld() { ++msm_time_tie_withheld_; }
  void msmSystemExcluded() { ++msm_system_excluded_; }
  void msmIncompleteDiscarded(size_t count) { msm_incomplete_discarded_+=count; }
  void msmChainTimeMismatch() { ++msm_chain_time_mismatch_; }
  void observationMergeConflict() { ++observation_merge_conflict_; }
  void observationNotMonotonic() { ++observation_not_monotonic_; }
  void stationFiltered() { ++station_filtered_; }
  void resetSession() { ++sessions_; std::memset(backend_.navigation,0,sizeof(backend_.navigation));
    std::memset(backend_.missing,0,sizeof(backend_.missing));
    std::memset(backend_.irn_seen,0,sizeof(backend_.irn_seen)); have_epoch_=false; last_period_=0; assembly_pending_=false; snapshot_rejected_=0; pvt_pending_=imu_pending_=false; }
  void expectObservations(double rate) { expected_rate_=rate; }
  void published(int week,double tow,size_t signals,Clock::time_point now=Clock::now()) {
    ++published_epochs_; published_signals_+=signals;
    if(!signals) { ++empty_epochs_; return; }
    const double delta=have_epoch_ ? (static_cast<double>(week)-week_)*604800.0+tow-tow_ : 0;
    last_period_=delta>0 ? delta : 0;
    week_=week; tow_=tow; last_epoch_=now; have_epoch_=true;
  }
  std::string describe(Clock::time_point now=Clock::now()) const {
    const char* state=!have_input_ ? "no_input" :
        std::chrono::duration<double>(now-last_input_).count()>period_ ? "input_stale" :
        !have_epoch_ ? "observations_not_observed" :
        std::chrono::duration<double>(now-last_epoch_).count()>period_ ? "observations_stale" : "observations_active";
    std::ostringstream s;
    s << std::setprecision(15);
    s << "Receiver health: state=" << state << " bytes=" << bytes_
      << " integrity_frames=" << frames_ << " framing_rejected=" << rejected_
      << " payload_errors=" << payload_errors_ << " decoded_epochs=" << decoded_epochs_
      << " published_epochs=" << published_epochs_ << " published_signals=" << published_signals_
      << " msm_split_detected=" << msm_split_detected_
      << " msm_time_tie_withheld=" << msm_time_tie_withheld_
      << " msm_system_excluded=" << msm_system_excluded_
      << " msm_incomplete_discarded=" << msm_incomplete_discarded_
      << " msm_chain_time_mismatch=" << msm_chain_time_mismatch_
      << " observation_merge_conflict=" << observation_merge_conflict_
      << " observation_not_monotonic=" << observation_not_monotonic_
      << " empty_epochs=" << empty_epochs_ << " unsupported_navigation=" << unsupported_navigation_
      << " station_filtered=" << station_filtered_ << " last_id=" << last_id_ << " last_revision=" << last_revision_
      << " last_epoch=";
    if(have_epoch_) s << week_ << ':' << tow_; else s << "unavailable";
    s << " last_period_s=";
    if(last_period_>0) s << last_period_; else s << "unavailable";
    s << " requested_obs_hz=";
    if(expected_rate_>0) s << expected_rate_; else s << "not_checked";
    s << " rate_matches_request=";
    if(expected_rate_>0 && last_period_>0)
      s << (std::abs(last_period_*expected_rate_-1)<=.05 ? "yes" : "no");
    else s << "not_checked";
    s << " sessions_reset=" << sessions_ << " backend_filter_counts=" << (backend_observed_ ? "instrumented" : "not_observed");
    s << " checksum_rejected=" << checksum_rejected_ << " malformed_candidates=" << malformed_
      << " incomplete_expired=" << incomplete_expired_ << " unknown_ids=" << unknown_ids_
      << " unknown_versions=" << unknown_versions_ << " version_unchecked_frames=" << version_unchecked_
      << " known_without_decoder_frames=" << known_without_decoder_
      << " last_rawx_version=";
    if(last_payload_version_>=0) s << last_payload_version_; else s << "unavailable";
    s << " maxobs=" << maxobs_ << " nfreq=" << nfreq_ << " nexobs=" << nexobs_ << " max_bds_prn=" << bds_prn_
      << " snapshot_rejected_records=" << snapshot_rejected_;
    s << " time_unresolved_frames=" << time_unresolved_ << " time_withheld_outputs=" << time_withheld_
      << " assembly_pending=" << (assembly_pending_ ? "yes" : "no") << " assembly_started=" << assembly_started_
      << " capacity_at_limit_epochs=" << capacity_at_limit_;
    s << " pvt_pending=" << (pvt_observed_ ? (pvt_pending_ ? "yes" : "no") : "not_observed")
      << " pvt_incomplete_epochs=" << pvt_incomplete_epochs_
      << " imu_pending=" << (imu_observed_ ? (imu_pending_ ? "yes" : "no") : "not_observed");
    static constexpr const char* names[GNSS_RX_EVENT_COUNT]={
      "satellite_exclusions", "signal_mapping_exclusions", "slot_selection_exclusions",
      "capacity_unread_rows", "capacity_rejected_insertions", "quality_exclusions", "selection_exclusions",
      "validation_events", "length_events", "time_events", "version_events", "reference_events",
      "navigation_no_output_events", "navigation_wait_events", "navigation_unsupported_events",
      "imu_wait_events", "imu_rejected_events", "frontend_rejected_events",
      "array_events", "field_events", "backend_checksum_events", "backend_station_events",
      "payload_unsupported_events", "navigation_duplicate_events", "navigation_quality_events",
      "navigation_page_ignored_events", "navigation_inconsistent_events", "navigation_auxiliary_events",
      "navigation_satellite_events", "value_events", "imu_scale_events", "assembly_replaced_events", "duplicate_input_events"};
    for(unsigned i=0;i<GNSS_RX_EVENT_COUNT;++i) {
      s << ' ' << names[i] << '=';
      if(!backend_observed_) s << "not_observed";
      else s << backend_.counts[i];
      if(backend_.functions[i])
        s << ' ' << names[i] << "_last=" << backend_.functions[i] << ':' << backend_.lines[i];
    }
    unsigned waiting=0,invalid=0,observed=0;
    std::ostringstream pending;
    static constexpr const char* families[]={"gps","inav","fnav","bds1","bds2","glo","irn"};
    for(unsigned sat=1;sat<GNSS_DIAGNOSTIC_SATELLITES;++sat)
      for(unsigned f=0;f<GNSS_NAV_FAMILIES;++f) {
        const auto state=backend_.navigation[sat][f];
        if(state) ++observed;
        if(state==2) ++waiting;
        if(state==3) ++invalid;
        if(state>=2) pending << (pending.tellp()>0 ? "," : "") << sat << ':' << families[f]
                            << ':' << (state==2 ? "wait" : "invalid") << ':' << backend_.missing[sat][f];
      }
    s << " navigation_pending=";
    if(observed) s << waiting; else s << "not_observed";
    s << " navigation_invalid=";
    if(observed) s << invalid; else s << "not_observed";
    s << " navigation_states=" << (pending.str().empty() ? (observed ? "none" : "not_observed") : pending.str());
    return s.str();
  }
  void report(const rclcpp::Logger& logger,Clock::time_point now=Clock::now()) {
    if(std::chrono::duration<double>(now-last_report_).count()<period_) return;
    last_report_=now;
    const auto line=describe(now);
    RCLCPP_INFO(logger,"%s",line.c_str());
  }
 private:
  gnss_receiver_diagnostics backend_{};
  bool backend_observed_{false},pvt_observed_{false},imu_observed_{false},pvt_pending_{false},imu_pending_{false};
  double period_{5},tow_{0},last_period_{0},expected_rate_{0}; int week_{0};
  uint64_t bytes_{0},frames_{0},rejected_{0},payload_errors_{0},decoded_epochs_{0};
  uint64_t published_epochs_{0},published_signals_{0},empty_epochs_{0},unsupported_navigation_{0},station_filtered_{0};
  uint64_t msm_split_detected_{0},msm_time_tie_withheld_{0},msm_system_excluded_{0},msm_incomplete_discarded_{0};
  uint64_t msm_chain_time_mismatch_{0},observation_merge_conflict_{0},observation_not_monotonic_{0};
  uint64_t sessions_{0},checksum_rejected_{0},malformed_{0},incomplete_expired_{0};
  uint64_t unknown_ids_{0},unknown_versions_{0},version_unchecked_{0},time_unresolved_{0},time_withheld_{0};
  uint64_t known_without_decoder_{0},pvt_incomplete_epochs_{0};
  uint64_t assembly_started_{0},capacity_at_limit_{0};
  bool assembly_pending_{false}; int last_payload_version_{-1};
  unsigned maxobs_{0},nfreq_{0},nexobs_{0},bds_prn_{0}; size_t snapshot_rejected_{0};
  unsigned last_id_{0},last_revision_{0}; bool have_input_{false},have_epoch_{false};
  Clock::time_point last_report_,last_input_{},last_epoch_{};
};
}
