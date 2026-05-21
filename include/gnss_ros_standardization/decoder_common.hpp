// SPDX-License-Identifier: MIT
#ifndef GNSS_ROS_STANDARDIZATION_DECODER_COMMON_HPP
#define GNSS_ROS_STANDARDIZATION_DECODER_COMMON_HPP

/// @file decoder_common.hpp
/// @brief Shared helpers used by the per-receiver decoder and driver nodes.
///
/// Receiver-agnostic glue logic (observation publication helpers, satellite
/// counting, ECEF↔LLH/ENU finalization, etc.) lives here so all
/// decoder/driver pairs share one implementation. Receiver-specific framing
/// and protocol parsing remain in `*_protocol.hpp`.

#include <vector>

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_observation.hpp"
#include "gnss_ros_standardization/msg/gnss_observations.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

extern "C" {
#include "rtklib.h"
}

namespace gnss_ros_standardization {
namespace decoder_common {

/// Per-constellation satellite counter, populated by `countSatellite`.
struct SatelliteCount {
  int gps{0};
  int glo{0};
  int gal{0};
  int qzs{0};
  int bds{0};
  int irn{0};
  int sbs{0};
  int unknown{0};
};

/// Classify an RTKLIB satellite number into the SatelliteCount bucket.
inline void countSatellite(int sat, SatelliteCount& c) {
  int prn = 0;
  switch (satsys(sat, &prn)) {
    case SYS_GPS: ++c.gps; break;
    case SYS_GLO: ++c.glo; break;
    case SYS_GAL: ++c.gal; break;
    case SYS_QZS: ++c.qzs; break;
    case SYS_CMP: ++c.bds; break;
    case SYS_IRN: ++c.irn; break;
    case SYS_SBS: ++c.sbs; break;
    default:      ++c.unknown; break;
  }
}

/// Append every non-empty (P / L / D / SNR) frequency channel of `obs` into
/// `observations` as a `GnssObservation` message.
inline void appendObservations(const obsd_t& obs,
                               std::vector<msg::GnssObservation>& observations) {
  for (int freq = 0; freq < NFREQ + NEXOBS; ++freq) {
    if (obs.P[freq] == 0.0 && obs.L[freq] == 0.0 &&
        obs.D[freq] == 0.0 && obs.SNR[freq] == 0) continue;
    observations.push_back(gnss_utils::obsToMsg(obs, freq));
  }
}

/// Derive ECEF position from LLH and ECEF velocity from ENU velocity using the
/// solution's current LLH as the reference frame. Used by every binary PVT
/// decoder/driver path after primary fields are populated.
inline void finalizeBinarySolutionGeometry(msg::GnssSolution& s) {
  double llh[3] = {s.latitude * D2R, s.longitude * D2R, s.altitude};
  double ecef[3] = {0};
  pos2ecef(llh, ecef);
  s.pos_ecef.x = ecef[0];
  s.pos_ecef.y = ecef[1];
  s.pos_ecef.z = ecef[2];

  double vel_e[3] = {s.vel_enu.x, s.vel_enu.y, s.vel_enu.z};
  double vel_ec[3] = {0};
  enu2ecef(llh, vel_e, vel_ec);
  s.vel_ecef.x = vel_ec[0];
  s.vel_ecef.y = vel_ec[1];
  s.vel_ecef.z = vel_ec[2];
}

}  // namespace decoder_common
}  // namespace gnss_ros_standardization

#endif  // GNSS_ROS_STANDARDIZATION_DECODER_COMMON_HPP
