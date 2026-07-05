// SPDX-License-Identifier: MIT
#ifndef GNSS_ROS_STANDARDIZATION_OBS_CONVERTER_HPP
#define GNSS_ROS_STANDARDIZATION_OBS_CONVERTER_HPP

#include <algorithm>
#include <map>
#include <vector>

#include "gnss_ros_standardization/msg/gnss_observation.hpp"
#include "gnss_ros_standardization/msg/gnss_observations.hpp"

extern "C" {
#include "rtklib.h"
}

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
  const int sat = satid2no(m.satid.c_str());
  if (sat <= 0 || sat > MAXSAT) return o;

  int prn = 0;
  const int sys = satsys(sat, &prn);
  const int idx = code2idx(sys, m.code);
  if (idx < 0 || idx >= NFREQ) return o;
  if (!isPrimaryCode(sys, m.code, idx)) return o;
  if (idx == 0 && !mask.l1) return o;
  if (idx == 1 && !mask.l2) return o;
  if (idx == 2 && !mask.l5) return o;

  o.time = t;
  o.sat = static_cast<uint8_t>(sat);
  o.rcv = static_cast<uint8_t>(rcv);
  o.P[idx] = m.p;
  o.L[idx] = m.l;
  o.D[idx] = static_cast<float>(m.d);
  o.SNR[idx] = static_cast<float>(m.snr);
  o.LLI[idx] = static_cast<uint8_t>(m.lli);
  o.code[idx] = m.code;
  return o;
}

// Build the RTKLIB observation vector for one epoch: convert every signal
// message, merge the per-band entries of the same (receiver, satellite) into
// one multi-frequency obsd_t, and sort by receiver then satellite — the
// layout rtkpos() expects (rover rcv=1 first, base rcv=2 after). Pass
// base == nullptr for rover-only epochs (SPP-style consumers).
inline std::vector<obsd_t> buildObsEpoch(
    const gnss_ros_standardization::msg::GnssObservations& rover,
    const gnss_ros_standardization::msg::GnssObservations* base,
    const FrequencyMask& mask) {
  std::map<int, obsd_t> obs_map;
  auto merge_obs = [&](const gnss_ros_standardization::msg::GnssObservation& m,
                       gtime_t t, int rcv) {
    const obsd_t o = convertObs(m, t, rcv, mask);
    if (o.sat == 0) return;
    const int key = (rcv << 16) | o.sat;
    auto it = obs_map.find(key);
    if (it == obs_map.end()) {
      obs_map[key] = o;
    } else {
      // Merge this signal into the existing (rcv, sat) entry. When two signals
      // map to the same band index (e.g. a receiver tracking both GPS L2L and
      // L2W), keep the band's primary slot for the higher-priority code, exactly
      // like RTKLIB set_index() ("assign index for highest priority code"). The
      // old "last signal wins" was order-dependent and, worse, could leave the
      // rover on a different L2 code than the base, so the double-difference's
      // code-equality check silently dropped the pair. Replace the band's fields
      // atomically (all from the same signal), not field-by-field.
      obsd_t& existing = it->second;
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
  for (const auto& kv : obs_map) obs.push_back(kv.second);

  std::sort(obs.begin(), obs.end(), [](const obsd_t& a, const obsd_t& b) {
    if (a.rcv != b.rcv) return a.rcv < b.rcv;
    return a.sat < b.sat;
  });
  return obs;
}

}  // namespace gnss_utils

#endif  // GNSS_ROS_STANDARDIZATION_OBS_CONVERTER_HPP
