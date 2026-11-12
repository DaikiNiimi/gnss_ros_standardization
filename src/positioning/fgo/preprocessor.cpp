#include "preprocessor.hpp"
#include <vector>

namespace gnss_fgo {
namespace {
// RTKLIB's SNR mask test (testsnr), so that the RTK and FGO nodes compare SNR
// with the same thresholds. In Preprocessor::edit a tracked band (seen with
// code and phase) below the mask leaves the whole satellite out for that
// receiver in that epoch; real_time_kinematic (RTKLIB zdres_sat) drops only
// that band.
bool snrMasked(const Config &cfg, int receiver_index, int band,
               double elevation_rad, double snr) {
  snrmask_t mask{};
  mask.ena[0] = cfg.snrmask_enable[0];
  mask.ena[1] = cfg.snrmask_enable[1];
  for (int b = 0; b < 3; ++b)
    for (int i = 0; i < 9; ++i)
      mask.mask[b][i] = cfg.snrmask[b][i];
  return testsnr(receiver_index, band, elevation_rad, snr, &mask) != 0;
}
double elevation(const Vec3 &receiver, const Vec3 &satellite) {
  constexpr double a = 6378137., f = 1. / 298.257223563, e2 = f * (2 - f);
  const double r2 = receiver.head<2>().squaredNorm();
  double z = receiver.z(), old = 0, v = a;
  while (std::abs(z - old) >= 1e-4) {
    old = z;
    double s = z / std::sqrt(r2 + z * z);
    v = a / std::sqrt(1 - e2 * s * s);
    z = receiver.z() + v * e2 * s;
  }
  const double lat = r2 > 1e-12 ? std::atan(z / std::sqrt(r2))
                                : (receiver.z() > 0 ? M_PI / 2 : -M_PI / 2);
  const double lon = r2 > 1e-12 ? std::atan2(receiver.y(), receiver.x()) : 0;
  const Vec3 up(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon),
                std::sin(lat));
  return std::asin(up.dot((satellite - receiver).normalized()));
}
gtime_t gtime(const PreciseTime &t) {
  gtime_t out = gpst2time(0, 0);
  out.time += t.seconds;
  out.sec = t.fraction;
  return out;
}
// RTKLIB satposs(), called with the ephemeris selection time RTK uses (the
// rover epoch) for both receivers. Missing or unhealthy satellites are left
// out.
std::map<int, SatelliteMotion> satelliteStates(const RawEpoch &epoch,
                                               const PreciseTime &rover_time,
                                               const nav_t &nav) {
  std::vector<obsd_t> obs;
  for (const auto &s : epoch.sats) {
    obsd_t o{};
    o.time = gtime(epoch.time);
    o.sat = s.sat;
    for (int f = 0; f < 3; ++f)
      o.P[f] = s.pr[f];
    obs.push_back(o);
  }
  std::map<int, SatelliteMotion> motions;
  const int n = int(obs.size());
  if (!n)
    return motions;
  std::vector<double> rs(6 * n), dts(2 * n), var(n);
  std::vector<int> svh(n);
  satposs(gtime(rover_time), obs.data(), n, &nav, EPHOPT_BRDC, rs.data(),
          dts.data(), var.data(), svh.data());
  for (int i = 0; i < n; ++i) {
    const Vec3 position(rs[6 * i], rs[6 * i + 1], rs[6 * i + 2]);
    if (position.isZero() || satexclude(obs[i].sat, var[i], svh[i], nullptr))
      continue;
    SatelliteMotion motion;
    motion.position = position;
    motion.velocity = Vec3(rs[6 * i + 3], rs[6 * i + 4], rs[6 * i + 5]);
    motion.clock_drift = dts[2 * i + 1];
    if (position.allFinite() && motion.velocity.allFinite())
      motions[obs[i].sat] = motion;
  }
  return motions;
}
} // namespace
std::map<int, Preprocessor::Edited>
Preprocessor::edit(const RawEpoch &obs, const Vec3 &position,
                   Receiver &receiver,
                   const std::map<int, SatelliteMotion> &motions,
                   int receiver_index) {
  std::map<int, Edited> result;
  for (const auto &s : obs.sats) {
    auto &out = result[s.sat];
    auto found = motions.find(s.sat);
    if (found == motions.end())
      continue;
    out.motion = found->second;
    out.elevation = elevation(position, out.motion.position);
    if (out.elevation < cfg_.elmin_deg * M_PI / 180.)
      continue;
    // A satellite is used on every band it has been seen with (code and phase)
    // or not at all: a seen band that is missing or below the SNR mask in this
    // epoch rejects the whole satellite for this receiver, so that a reflected
    // signal does not reach the solution through the satellite's other band.
    // A band never seen is not required; it is edited (code or phase is 0).
    bool seen_any = false, rejected = false;
    for (int f = 0; f < 3; ++f) {
      out.slip[f] = s.lli[f] & 1;
      out.edited[f] =
          s.pr[f] == 0 || s.phase[f] == 0 ||
          snrMasked(cfg_, receiver_index, f, out.elevation, s.snr[f]);
      const SF band{s.sat, f};
      if (s.pr[f] != 0 && s.phase[f] != 0)
        receiver.seen.insert(band);
      if (receiver.seen.count(band)) {
        seen_any = true;
        rejected = rejected || out.edited[f];
      }
    }
    out.admitted = seen_any && !rejected;
    if (!out.admitted)
      out.edited.fill(true);
    if (s.phase[0] != 0 && s.phase[1] != 0) {
      const double gf = s.phase[0] * s.wavelength[0] -
                        s.phase[1] * s.wavelength[1],
                   old = receiver.gf[s.sat];
      if (gf != 0)
        receiver.gf[s.sat] = gf;
      if (gf != 0 && old != 0 && std::abs(gf - old) > cfg_.thres_slip)
        out.slip[0] = out.slip[1] = true;
    }
  }
  return result;
}
void Preprocessor::continuity(const RawEpoch &obs, Receiver &receiver,
                              const std::map<int, Edited> &edited) {
  for (const auto &s : obs.sats)
    for (int f = 0; f < 3; ++f) {
      if (s.phase[f] == 0)
        continue;
      const SF sf{s.sat, f};
      auto qc = edited.find(s.sat);
      const bool slipped =
          (s.lli[f] & 1) || (qc != edited.end() && qc->second.slip[f]);
      const auto event = std::make_tuple(obs.time.seconds, obs.time.fraction,
                                         s.signals[f], slipped);
      const auto prev = receiver.continuity.find(sf);
      if (prev == receiver.continuity.end() || prev->second != event) {
        if (slipped || (prev != receiver.continuity.end() &&
                        std::get<2>(prev->second) != s.signals[f]))
          pending_.insert(sf);
        receiver.continuity[sf] = event;
      }
    }
}
ObservationEpoch Preprocessor::prepare(const RawEpoch &rover,
                                       const std::optional<RawEpoch> &base,
                                       const Vec3 &position) {
  if (!position.allFinite())
    throw std::invalid_argument("invalid preprocessing position");
  ObservationEpoch out;
  out.gpst = rover.time.absolute();
  out.base_gpst = base ? base->time.absolute() : out.gpst;
  std::map<int, SatelliteMotion> ru, rb;
  std::map<int, Edited> eu, eb;
  if (base && !rover.sats.empty() && !base->sats.empty()) {
    ru = satelliteStates(rover, rover.time, nav_);
    if (ru.size() >= 4) {
      rb = satelliteStates(*base, rover.time, nav_);
      eb = edit(*base, base_, base_receiver_, rb, 1);
      eu = edit(rover, position, rover_, ru, 0);
    }
  }
  continuity(rover, rover_, eu);
  if (base)
    continuity(*base, base_receiver_, eb);
  if (base) {
    std::map<int, const RawSatellite *> bs;
    for (const auto &s : base->sats)
      bs[s.sat] = &s;
    for (const auto &s : rover.sats) {
      if (!bs.count(s.sat) || !eu.count(s.sat) || !eb.count(s.sat) ||
          !eu.at(s.sat).admitted || !eb.at(s.sat).admitted)
        continue;
      const auto &b = *bs.at(s.sat);
      const auto &uqc = eu.at(s.sat);
      const auto &bqc = eb.at(s.sat);
      SatelliteObservation v;
      v.sat = s.sat;
      v.system = s.system;
      v.prn = s.prn;
      v.elevation = uqc.elevation;
      v.rover_sat = uqc.motion.position;
      v.sat_velocity = uqc.motion.velocity;
      v.sat_clock_drift = uqc.motion.clock_drift;
      v.base_sat = bqc.motion.position;
      v.pr = s.pr;
      v.pr_base = b.pr;
      v.phase = s.phase;
      v.phase_base = b.phase;
      v.doppler = s.doppler;
      for (int f = 0; f < 3; ++f) {
        if (uqc.edited[f])
          v.pr[f] = v.phase[f] = v.doppler[f] = 0;
        if (bqc.edited[f])
          v.pr_base[f] = v.phase_base[f] = 0;
      }
      v.doppler_wavelength = s.doppler_wavelength;
      v.doppler_snr = s.doppler_snr;
      v.snr = s.snr;
      v.wavelength = s.wavelength;
      v.lli = s.lli;
      out.sats.push_back(v);
    }
  }
  std::sort(out.sats.begin(), out.sats.end(),
            [](const auto &a, const auto &b) { return a.sat < b.sat; });
  if (out.sats.size() < 4)
    out.sats.clear();
  out.input_slips = std::move(pending_);
  pending_.clear();
  return out;
}
} // namespace gnss_fgo
