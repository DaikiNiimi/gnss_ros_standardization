#include "preprocessor.hpp"

namespace gnss_fgo {
namespace {
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
} // namespace
std::map<int, Preprocessor::Edited>
Preprocessor::edit(const RawEpoch &obs, const Vec3 &position,
                   Receiver &receiver,
                   const std::map<int, SatelliteMotion> &motions) {
  std::map<int, Edited> result;
  for (const auto &s : obs.sats) {
    auto &out = result[s.sat];
    auto found = motions.find(s.sat);
    if (found == motions.end())
      continue;
    out.motion = found->second;
    if (!out.motion.available || out.motion.health > 0)
      continue;
    out.elevation = elevation(position, out.motion.position);
    if (out.elevation < cfg_.elmin_deg * M_PI / 180.)
      continue;
    for (int f = 0; f < 3; ++f)
      if (s.phase[f] != 0 && s.pr[f] != 0)
        receiver.seen[{s.sat, f}] = true;
    for (int f = 0; f < s.bands; ++f) {
      out.slip[f] = s.lli[f] & 1;
      out.edited[f] = s.pr[f] == 0 || s.phase[f] == 0 ||
                      s.snr[f] < (s.gps_py[f] ? 15. : cfg_.cnr_min_dbhz);
    }
    if (s.bands > 1 && s.phase[0] != 0 && s.phase[1] != 0) {
      const double gf = s.phase[0] * s.wavelength[0] -
                        s.phase[1] * s.wavelength[1],
                   old = receiver.gf[s.sat];
      if (gf != 0)
        receiver.gf[s.sat] = gf;
      if (gf != 0 && old != 0 && std::abs(gf - old) > .15)
        out.slip[0] = out.slip[1] = true;
    }
    bool any = false, bad = false;
    for (int f = 0; f < s.bands; ++f) {
      const bool judged = !cfg_.sat_band_plan || receiver.seen[{s.sat, f}];
      if (judged) {
        any = true;
        bad = bad || out.edited[f];
      } else
        out.edited[f] = true;
    }
    out.admitted = any && !bad;
    if (!out.admitted)
      out.edited.fill(true);
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
  // Pairs arrive in rover order with non-decreasing bases, so no later epoch
  // propagates before this pair's earlier time (60 s margin).
  navigation.prune(
      (base && base->time - rover.time < 0 ? base->time : rover.time)
          .add(-60.));
  ObservationEpoch out;
  out.gpst = rover.time.absolute();
  out.base_gpst = base ? base->time.absolute() : out.gpst;
  std::map<int, SatelliteMotion> ru, rb;
  std::map<int, Edited> eu, eb;
  auto propagate = [&](const RawEpoch &obs,
                       std::map<int, SatelliteMotion> &motions) {
    for (const auto &s : obs.sats) {
      if (s.system < 0 || s.system > 3)
        continue;
      double range = 0;
      for (int f = 0; f < 3; ++f)
        if (s.pr[f] != 0) {
          range = s.pr[f];
          break;
        }
      auto motion =
          navigation.position(s.sat, s.system, obs.time, range, rover.time);
      if (motion.available)
        motions[s.sat] = motion;
    }
  };
  if (base && !rover.sats.empty() && !base->sats.empty()) {
    propagate(rover, ru);
    if (ru.size() >= 4) {
      propagate(*base, rb);
      eb = edit(*base, base_, base_receiver_, rb);
      eu = edit(rover, position, rover_, ru);
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
      v.base_sat = bqc.motion.position;
      v.pr = s.pr;
      v.pr_base = b.pr;
      v.phase = s.phase;
      v.phase_base = b.phase;
      for (int f = 0; f < 3; ++f) {
        if (uqc.edited[f])
          v.pr[f] = v.phase[f] = 0;
        if (bqc.edited[f])
          v.pr_base[f] = v.phase_base[f] = 0;
      }
      v.doppler = s.doppler;
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
