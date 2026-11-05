#pragma once
// Broadcast-ephemeris propagation and receiver QC/continuity (GNSS frontend).
#include "model.hpp"
#include <array>
#include <cstdint>
#include <iterator>
#include <map>

namespace gnss_fgo {
// Keep fractional seconds separate during transmission-time propagation.
// A double absolute GPST loses enough precision to move a satellite by mm.
struct PreciseTime {
  std::int64_t seconds = 0;
  double fraction = 0;
  double absolute() const { return double(seconds) + fraction; }
  double operator-(const PreciseTime &other) const {
    return double(seconds - other.seconds) + (fraction - other.fraction);
  }
  PreciseTime add(double delta) const {
    const double f = fraction + delta, whole = std::floor(f);
    return {seconds + std::int64_t(whole), f - whole};
  }
};
struct BroadcastEphemeris {
  int id = 0, sat = 0, system = 0, prn = 0, mode = 0, health = 0;
  PreciseTime toe, toc, transmitted;
  double toes = 0, A = 0, e = 0, M0 = 0, omg = 0, i0 = 0, OMG0 = 0, OMGd = 0;
  double deln = 0, delnd = 0, Adot = 0, idot = 0, crc = 0, crs = 0, cuc = 0,
         cus = 0, cic = 0, cis = 0;
  double af0 = 0, af1 = 0, af2 = 0;
};
struct SatelliteMotion {
  Vec3 position = Vec3::Zero(), velocity = Vec3::Zero();
  double clock = 0;
  int health = 1;
  bool available = false;
};
inline double weekAdjusted(double dt) {
  if (dt > 604800.)
    dt -= 604800.;
  else if (dt < -604800.)
    dt += 604800.;
  return dt;
}
inline double broadcastClock(const PreciseTime &t,
                             const BroadcastEphemeris &e) {
  double dt = t - e.toc;
  for (int i = 0; i < 2; ++i)
    dt -= e.af0 + e.af1 * dt + e.af2 * dt * dt;
  return e.af0 + e.af1 * dt + e.af2 * dt * dt;
}
inline SatelliteMotion broadcastOrbit(const PreciseTime &t,
                                      const BroadcastEphemeris &e) {
  constexpr double light = 299792458.;
  const double mu =
      (e.system == 1 || e.system == 3) ? 3.986004418e14 : 3.9860050e14;
  const double omega = e.system == 3 ? 7.2921150e-5 : 7.2921151467e-5;
  const double dt = weekAdjusted(t - e.toe), dtc = weekAdjusted(t - e.toc);
  const double nd = e.mode > 0 ? .5 * e.delnd * dt : 0.,
               Akd = e.mode > 0 ? e.Adot : 0.;
  const double Ak = e.A + Akd * dt,
               n = std::sqrt(mu / std::pow(e.A, 3)) + e.deln + nd;
  const double M = e.M0 + n * dt;
  double E = M;
  for (int i = 0; i < 30; ++i) {
    double old = E;
    E = M + e.e * std::sin(E);
    if (std::abs(E - old) < 1e-13)
      break;
  }
  const double sE = std::sin(E), cE = std::cos(E), nue = 1. - e.e * cE,
               es = std::sqrt(1. - e.e * e.e);
  const double phi = std::atan2(es * sE, cE - e.e) + e.omg,
               c2 = std::cos(2 * phi), s2 = std::sin(2 * phi);
  const double u = phi + e.cuc * c2 + e.cus * s2,
               r = Ak * nue + e.crc * c2 + e.crs * s2;
  const double inc = e.i0 + e.idot * dt + e.cic * c2 + e.cis * s2,
               si = std::sin(inc), ci = std::cos(inc);
  const double su = std::sin(u), cu = std::cos(u), xo0 = r * cu, xo1 = r * su;
  const bool geo = e.system == 3 && (e.prn <= 5 || e.prn >= 59);
  const double O = e.OMG0 + e.OMGd * dt - omega * (geo ? e.toes : e.toes + dt),
               sO = std::sin(O), cO = std::cos(O);
  const Vec3 p(cO, sO, 0), q(-ci * sO, ci * cO, si);
  SatelliteMotion out;
  out.position = xo0 * p + xo1 * q;
  out.clock = e.af0 + e.af1 * dtc + e.af2 * dtc * dtc -
              2 * std::sqrt(mu * e.A) * e.e * sE / (light * light);
  const double Ed = (n + nd) / nue, nud = es / nue * Ed, h0 = -2 * nud * s2,
               h1 = 2 * nud * c2;
  const double ud = nud + e.cuc * h0 + e.cus * h1,
               rd = Akd * nue + Ak * e.e * sE * Ed + e.crc * h0 + e.crs * h1;
  const double xod0 = rd * cu - r * ud * su, xod1 = rd * su + r * ud * cu,
               incd = e.idot + e.cic * h0 + e.cis * h1;
  const double od = geo ? e.OMGd : e.OMGd - omega;
  const Vec3 pd = Vec3(-p.y(), p.x(), 0) * od,
             qd = Vec3(-q.y(), q.x(), 0) * od +
                  Vec3(si * sO, -si * cO, ci) * incd;
  out.velocity = pd * xo0 + qd * xo1 + p * xod0 + q * xod1;
  if (geo) {
    constexpr double c5 = .9961946980917456, s5 = -.0871557427476582;
    const double so = std::sin(omega * dt), co = std::cos(omega * dt);
    Mat3 rot, rate;
    rot << co, so * c5, so * s5, -so, co * c5, co * s5, 0, -s5, c5;
    rate << -so, co * c5, co * s5, -co, -so * c5, -so * s5, 0, 0, 0;
    rate *= omega;
    out.velocity = (rot * out.velocity + rate * out.position).eval();
    out.position = (rot * out.position).eval();
  }
  out.health = e.health;
  out.available = out.position.allFinite() && out.velocity.allFinite() &&
                  std::isfinite(out.clock);
  return out;
}
class BroadcastStore {
public:
  void insert(const BroadcastEphemeris &e) {
    if (e.system < 0 || e.system > 3)
      throw std::invalid_argument("unsupported broadcast constellation");
    // Some received QZSS NAV records explicitly mark an unhealthy satellite
    // and contain no orbit. Retain their identity/health instead of aborting
    // the complete receiver stream or falling back to an older healthy record.
    // A republished identical record (snapshot heartbeat) replaces its older
    // copy: the newer id would win every tie, so the older copy can never be
    // selected again. Keeps the store bounded under periodic snapshots.
    auto &records = records_[e.sat];
    for (auto it = records.begin(); it != records.end(); ++it)
      if (it->first < e.id && sameContent(it->second, e)) {
        records.erase(it);
        break;
      }
    records[e.id] = e;
  }
  // Retire records that no later epoch can select: an epoch at or after
  // `earliest` transmits after it, and a record more than max_age from every
  // such time is outside the selection window. Callers pass a time no later
  // than any epoch (rover or base) still to be propagated.
  void prune(const PreciseTime &earliest) {
    for (auto sat = records_.begin(); sat != records_.end();) {
      for (auto it = sat->second.begin(); it != sat->second.end();) {
        const double max_age = it->second.system == 1 ? 14400. : 7201.;
        if (earliest - it->second.toe > max_age + 1.)
          it = sat->second.erase(it);
        else
          ++it;
      }
      sat = sat->second.empty() ? records_.erase(sat) : std::next(sat);
    }
  }
  std::size_t size() const {
    std::size_t n = 0;
    for (const auto &[sat, records] : records_)
      n += records.size();
    return n;
  }
  SatelliteMotion position(int sat, int system, const PreciseTime &receive,
                           double range, const PreciseTime &admitted) const {
    auto found = records_.find(sat);
    if (found == records_.end() || range == 0)
      return {};
    const PreciseTime transmit = receive.add(-range / 299792458.);
    const double max_age = system == 1 ? 14400. : 7201.;
    const BroadcastEphemeris *best = nullptr;
    for (int fallback = 0; fallback < 2 && !best; ++fallback) {
      double min_age = max_age + 1;
      for (const auto &[id, e] : found->second) {
        if (e.transmitted - admitted > 0 || (!fallback && e.mode != 0))
          continue;
        const double age = std::abs(transmit - e.toe);
        if (age <= max_age && age <= min_age) {
          best = &e;
          min_age = age;
        }
      }
    }
    if (!best)
      return {};
    if (!(best->A > 0 && std::isfinite(best->A) && best->e >= 0 && best->e < 1))
      return {};
    return broadcastOrbit(transmit.add(-broadcastClock(transmit, *best)),
                          *best);
  }

private:
  static bool sameTime(const PreciseTime &a, const PreciseTime &b) {
    return a.seconds == b.seconds && a.fraction == b.fraction;
  }
  static bool sameContent(const BroadcastEphemeris &a,
                          const BroadcastEphemeris &b) {
    return a.sat == b.sat && a.system == b.system && a.prn == b.prn &&
           a.mode == b.mode && a.health == b.health && sameTime(a.toe, b.toe) &&
           sameTime(a.toc, b.toc) && sameTime(a.transmitted, b.transmitted) &&
           a.toes == b.toes && a.A == b.A && a.e == b.e && a.M0 == b.M0 &&
           a.omg == b.omg && a.i0 == b.i0 && a.OMG0 == b.OMG0 &&
           a.OMGd == b.OMGd && a.deln == b.deln && a.delnd == b.delnd &&
           a.Adot == b.Adot && a.idot == b.idot && a.crc == b.crc &&
           a.crs == b.crs && a.cuc == b.cuc && a.cus == b.cus &&
           a.cic == b.cic && a.cis == b.cis && a.af0 == b.af0 &&
           a.af1 == b.af1 && a.af2 == b.af2;
  }
  // Preserve received record IDs: equally near records use the final entry.
  std::map<int, std::map<int, BroadcastEphemeris>> records_;
};
} // namespace gnss_fgo

namespace gnss_fgo {
struct RawSatellite {
  int sat = 0, system = 0, prn = 0, bands = 0;
  Vec3 pr = Vec3::Zero(), phase = Vec3::Zero(), doppler = Vec3::Zero(),
       snr = Vec3::Zero(), wavelength = Vec3::Zero();
  Vec3 doppler_wavelength = Vec3::Zero(), doppler_snr = Vec3::Zero();
  Eigen::Vector3i lli = Eigen::Vector3i::Zero();
  std::array<std::string, 3> signals;
  std::array<bool, 3> gps_py{};
};
struct RawEpoch {
  PreciseTime time;
  std::vector<RawSatellite> sats;
};
class Preprocessor {
public:
  Preprocessor(const Config &config, const Vec3 &base)
      : cfg_(config), base_(base) {}
  BroadcastStore navigation;
  ObservationEpoch prepare(const RawEpoch &rover,
                           const std::optional<RawEpoch> &base,
                           const Vec3 &position);

private:
  struct Receiver {
    std::map<SF, bool> seen;
    std::map<int, double> gf;
    std::map<SF, std::tuple<std::int64_t, double, std::string, bool>>
        continuity;
  };
  struct Edited {
    SatelliteMotion motion;
    double elevation = 0;
    bool admitted = false;
    std::array<bool, 3> edited{true, true, true}, slip{};
  };
  Config cfg_;
  Vec3 base_;
  Receiver rover_, base_receiver_;
  std::set<SF> pending_;
  std::map<int, Edited> edit(const RawEpoch &, const Vec3 &, Receiver &,
                             const std::map<int, SatelliteMotion> &);
  void continuity(const RawEpoch &, Receiver &, const std::map<int, Edited> &);
};
} // namespace gnss_fgo
