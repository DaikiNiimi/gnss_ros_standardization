#ifndef GNSS_ROS_STANDARDIZATION_OBS_CONVERTER_HPP
#define GNSS_ROS_STANDARDIZATION_OBS_CONVERTER_HPP

#include <algorithm>
#include <array>
#include <iterator>
#include <cmath>
#include "gnss_ros_standardization/gnss_utils.hpp"
#include <map>
#include <set>
#include <vector>

#include "gnss_ros_standardization/msg/gnss_observation.hpp"
#include "gnss_ros_standardization/msg/gnss_observations.hpp"

#include "gnss_ros_standardization/rtklib_compat.hpp"

namespace gnss_utils {

// Per-band enable flags applied during message-to-obsd_t conversion.
struct FrequencyMask {
  bool l1{true};
  bool l2{true};
  bool l5{false};
};

// Guard against frequency collisions in code2idx(). The only true collision
// is BDS idx=0 (B1I vs B1C). Other systems can safely accept any code.
inline bool isPrimaryCode(int sys, uint8_t code, int idx) {
  if (sys == SYS_CMP && idx == 0 && code != CODE_L2I) return false;
  return true;
}

// Convert one GnssObservation message (one satellite, one signal) into an
// obsd_t with the band placed at its RTKLIB frequency index. Returns an
// obsd_t with sat == 0 when the signal is rejected (unknown satellite,
// unknown/colliding code, or band disabled by the mask).
inline obsd_t convertObs(const gnss_ros_standardization::msg::GnssObservation& m,
                         gtime_t t, int rcv, const FrequencyMask& mask) {
  obsd_t o{};
  const int sat = resolveSatellite(m.satid, m.system, m.prn);
  if (sat <= 0 || sat > MAXSAT) return o;

  int prn = 0;
  const int sys = satsys(sat, &prn);
  const int code = resolveSignal(m.code_str, m.code);
  if (!code) return o;
  const int idx = code2idx(sys, code);
  if (idx < 0 || idx >= NFREQ) return o;
  if (!isPrimaryCode(sys, code, idx)) return o;
  if (idx == 0 && !mask.l1) return o;
  if (idx == 1 && !mask.l2) return o;
  if (idx == 2 && !mask.l5) return o;

  o.time = t;
  o.sat = static_cast<uint8_t>(sat);
  o.rcv = static_cast<uint8_t>(rcv);
  o.freq = sys == SYS_GLO && m.freq <= 13 ? m.freq : 255;
  o.P[idx] = std::isfinite(m.p) && m.p >= 0 ? m.p : 0;
  o.L[idx] = std::isfinite(m.l) ? m.l : 0;
  o.D[idx] = std::isfinite(m.d) && std::abs(m.d) <= std::numeric_limits<float>::max() ? static_cast<float>(m.d) : 0;
  o.SNR[idx] = std::isfinite(m.snr) && m.snr >= 0 ? m.snr : 0;
  o.LLI[idx] = static_cast<uint8_t>(m.lli);
  o.code[idx] = code;
  return o;
}

// Existing consumer compatibility policy. This does not relabel ROS signals;
// FGO still applies its cross_code_carrier setting before forming differences.
inline bool crossCodePairingSupported(int sys, int band) {
  return sys==SYS_GAL || (band==2 && (sys==SYS_GPS || sys==SYS_QZS));
}

// Private solver state: retain one code per constellation/band across epochs.
// This is not RTKLIB's per-file RINEX selection. Prefer a shared code; retain
// established Galileo/L5 component pairing where no exact common code exists.
// A change starts a new carrier arc on both private receiver copies.
class SignalSelection {
 public:
  void reset() { selected_.clear(); arcs_.clear(); }
  std::map<std::pair<int,int>, std::array<uint8_t,2>> selected_;
  std::map<std::pair<int,int>, std::array<uint8_t,2>> arcs_;
};

// Build the RTKLIB observation vector for one epoch: convert every signal
// message, merge the per-band entries of the same (receiver, satellite) into
// one multi-frequency obsd_t, and sort by receiver then satellite — the
// layout rtkpos() expects (rover rcv=1 first, base rcv=2 after). Pass
// base == nullptr for rover-only epochs (SPP-style consumers).
inline std::vector<obsd_t> buildObsEpoch(
    const gnss_ros_standardization::msg::GnssObservations& rover,
    const gnss_ros_standardization::msg::GnssObservations* base,
    const FrequencyMask& mask, ObservationRejectCounters* counters = nullptr,
    SignalSelection* selection = nullptr) {
  auto valid_time = [](const auto& epoch) {
    return epoch.week <= static_cast<uint32_t>(std::numeric_limits<int>::max()) &&
           std::isfinite(epoch.tow) && epoch.tow >= 0 && epoch.tow < 604800;
  };
  if (!valid_time(rover) || (base && !valid_time(*base))) {
    if (counters) ++counters->invalid_epoch;
    return {};
  }
  // RTKLIB differences satellites within a constellation. Selecting different
  // tracking families per satellite changes the inter-receiver bias within
  // that group. Use a common group policy, not a per-satellite fallback.
  using Band = std::pair<int,int>; // (constellation, frequency index)
  using Codes = std::set<uint8_t>;
  std::map<Band, std::array<Codes,2>> candidates;
  std::map<Band, std::array<Codes,2>> per_satellite;
  std::map<Band, std::array<uint8_t,2>> chosen;
  std::set<Band> switched;
  if (selection) {
    auto collect = [&](const auto& epoch, int rcv) {
      for (const auto& m : epoch.observations) {
        const auto o = convertObs(m, gpst2time(epoch.week, epoch.tow), rcv, mask);
        if (!o.sat) continue;
        for (int band=0; band<NFREQ; ++band)
          if (o.code[band]) per_satellite[{o.sat,band}][rcv-1].insert(o.code[band]);
      }
    };
    collect(rover,1);
    if (base) collect(*base,2);
    for (const auto& [sat_band, receivers]:per_satellite) {
      if (receivers[0].empty() || receivers[1].empty()) continue;
      const Band group{satsys(sat_band.first,nullptr),sat_band.second};
      for (int rcv=0;rcv<2;++rcv)
        candidates[group][rcv].insert(receivers[rcv].begin(),receivers[rcv].end());
    }
    std::map<Band,std::array<uint8_t,2>> group_codes;
    for (const auto& [key, receivers] : candidates) {
      const int sys=key.first;
      const auto previous=selection->selected_.find(key);
      const std::array<uint8_t,2> old=previous==selection->selected_.end()
          ? std::array<uint8_t,2>{} : previous->second;
      const auto pick=[&](const Codes& allowed,uint8_t sticky) -> uint8_t {
        if (allowed.empty()) return 0;
        if (allowed.count(sticky)) return sticky;
        uint8_t code=*allowed.begin();
        for (const auto candidate:allowed)
          if (getcodepri(sys,candidate,"")>getcodepri(sys,code,"")) code=candidate;
        return code;
      };
      std::array<uint8_t,2> codes{};
      if (!receivers[0].empty() && !receivers[1].empty()) {
        Codes common;
        std::set_intersection(receivers[0].begin(),receivers[0].end(),
                              receivers[1].begin(),receivers[1].end(),
                              std::inserter(common,common.end()));
        if (!common.empty()) {
          codes.fill(pick(common,common.count(old[0]) ? old[0] : old[1]));
        } else if (old[0] && old[0]==old[1] &&
                   (receivers[0].count(old[0]) || receivers[1].count(old[0]))) {
          // A temporary outage is not a tracking-mode change. Keep the
          // established code on the receiver that still observes it. There
          // is no mixed-code DD, and no artificial hole in its phase history.
          codes=old;
        } else if (crossCodePairingSupported(sys,key.second)) {
          // Preserve the pre-existing Galileo/L5 component-pairing policy.
          // Exact-code-only selection would remove every Galileo band in
          // heterogeneous PPC receivers (rover C/Q, base X).
          for (int rcv=0;rcv<2;++rcv) codes[rcv]=pick(receivers[rcv],old[rcv]);
        } else if ((sys==SYS_GPS || sys==SYS_QZS) && key.second==1) {
          // RTKLIB permits stable heterogeneous L2 tracking. Select once per
          // constellation/receiver; a real change is still a private slip.
          for (int rcv=0;rcv<2;++rcv) codes[rcv]=pick(receivers[rcv],old[rcv]);
        } else continue; // No compatible pair: preserve prior selection through outage.
      } else {
        for (int rcv=0;rcv<2;++rcv) codes[rcv]=pick(receivers[rcv],old[rcv]);
      }
      auto next=old;
      for (int rcv=0;rcv<2;++rcv) if (codes[rcv]) {
        next[rcv]=codes[rcv];
      }
      group_codes[key]=codes;
      selection->selected_[key]=next;
    }
    for (const auto& [key, receivers]:per_satellite) {
      const int sys=satsys(key.first,nullptr);
      auto old=selection->arcs_[key];
      std::array<uint8_t,2> codes{};
      if (!receivers[0].empty() && !receivers[1].empty()) {
        const auto group=group_codes.find({sys,key.second});
        if (group!=group_codes.end()) codes=group->second;
      } else {
        // A receiver-only band cannot form a double difference. Retain its
        // independent SPP/Doppler information, including BDS B2b on the base.
        for (int rcv=0;rcv<2;++rcv) if (!receivers[rcv].empty()) {
          codes[rcv]=*receivers[rcv].begin();
          if (receivers[rcv].count(old[rcv])) codes[rcv]=old[rcv];
          else for (auto code:receivers[rcv])
            if (getcodepri(sys,code,"")>getcodepri(sys,codes[rcv],"")) codes[rcv]=code;
        }
      }
      for (int rcv=0;rcv<2;++rcv) {
        if (!receivers[rcv].count(codes[rcv])) { codes[rcv]=0; continue; }
        if (old[rcv] && old[rcv]!=codes[rcv]) switched.insert(key);
        old[rcv]=codes[rcv];
      }
      selection->arcs_[key]=old;
      chosen[key]=codes;
    }
  }
  std::map<int, obsd_t> obs_map;
  std::set<int> conflicting_channels;
  std::map<int,uint8_t> known_channels;
  auto merge_obs = [&](const gnss_ros_standardization::msg::GnssObservation& m,
                       gtime_t t, int rcv) {
    obsd_t o = convertObs(m, t, rcv, mask);
    if (o.sat == 0) return;
    const int key = (rcv << 16) | o.sat;
    if (o.freq<=13) {
      const auto inserted=known_channels.emplace(key,o.freq);
      if (!inserted.second && inserted.first->second!=o.freq) conflicting_channels.insert(key);
    }
    if (selection) {
      bool accepted = false;
      for (int band=0; band<NFREQ; ++band) {
        if (!o.code[band]) continue;
        const Band key{o.sat,band};
        const auto it=chosen.find(key);
        if (it==chosen.end() || it->second[rcv-1]!=o.code[band]) return;
        if (switched.count(key)) o.LLI[band] |= LLI_SLIP;
        accepted = true;
      }
      if (!accepted) return;
    }
    auto it = obs_map.find(key);
    if (it == obs_map.end()) {
      obs_map[key] = o;
    } else {
      // Atomically merge one chosen signal into its frequency slot. Without
      // a stateful selector, preserve the legacy per-epoch priority behavior.
      obsd_t& existing = it->second;
      if (existing.freq <= 13 && o.freq <= 13 && existing.freq != o.freq)
        conflicting_channels.insert(key);
      // A missing channel in one signal must not erase another signal's FCN.
      if (existing.freq == 255) existing.freq = o.freq;
      int prn = 0;
      const int sys = satsys(o.sat, &prn);
      for (int i = 0; i < NFREQ; ++i) {
        if (o.code[i] == 0) continue;  // band not populated by this signal
        if (existing.code[i] != 0 &&
            getcodepri(sys, o.code[i], "") <=
                getcodepri(sys, existing.code[i], "")) {
          continue;  // keep the existing code (priority >= incoming)
        }
        existing.P[i] = o.P[i];
        existing.L[i] = o.L[i];
        existing.D[i] = o.D[i];
        existing.SNR[i] = o.SNR[i];
        existing.LLI[i] = o.LLI[i];
        existing.code[i] = o.code[i];
      }
    }
  };

  const gtime_t tr = gpst2time(static_cast<int>(rover.week), rover.tow);
  for (const auto& m : rover.observations) merge_obs(m, tr, 1);
  if (base) {
    const gtime_t tb = gpst2time(static_cast<int>(base->week), base->tow);
    for (const auto& m : base->observations) merge_obs(m, tb, 2);
  }

  std::vector<obsd_t> obs;
  obs.reserve(obs_map.size());
  if (counters) counters->signal_channel_conflict += conflicting_channels.size();
  for (const auto& kv : obs_map) {
    if (!conflicting_channels.count(kv.first)) {
      auto value=kv.second;
      const auto channel=known_channels.find(kv.first);
      if (channel!=known_channels.end()) value.freq=channel->second;
      obs.push_back(value);
    }
  }

  std::sort(obs.begin(), obs.end(), [](const obsd_t& a, const obsd_t& b) {
    if (a.rcv != b.rcv) return a.rcv < b.rcv;
    return a.sat < b.sat;
  });
  return obs;
}

}  // namespace gnss_utils

#endif  // GNSS_ROS_STANDARDIZATION_OBS_CONVERTER_HPP
