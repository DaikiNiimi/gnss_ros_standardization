#pragma once
// Satellite states from RTKLIB and receiver QC/continuity (GNSS frontend).
#include "model.hpp"
#include "gnss_ros_standardization/rtklib_compat.hpp"
#include <array>
#include <cstdint>
#include <map>

namespace gnss_fgo {
// GPST as integer seconds plus fraction, converted to RTKLIB gtime_t for
// satposs().
struct PreciseTime {
  std::int64_t seconds = 0;
  double fraction = 0;
  double absolute() const { return double(seconds) + fraction; }
};
struct SatelliteMotion {
  Vec3 position = Vec3::Zero(), velocity = Vec3::Zero();
  double clock_drift = 0;
};
} // namespace gnss_fgo

namespace gnss_fgo {
struct RawSatellite {
  int sat = 0, system = 0, prn = 0;
  Vec3 pr = Vec3::Zero(), phase = Vec3::Zero(), doppler = Vec3::Zero(),
       snr = Vec3::Zero(), wavelength = Vec3::Zero();
  Vec3 doppler_wavelength = Vec3::Zero(), doppler_snr = Vec3::Zero();
  Eigen::Vector3i lli = Eigen::Vector3i::Zero();
  std::array<std::string, 3> signals;
};
struct RawEpoch {
  PreciseTime time;
  std::vector<RawSatellite> sats;
};
class Preprocessor {
public:
  Preprocessor(const Config &config, const Vec3 &base)
      : cfg_(config), base_(base) {}
  ~Preprocessor() { freenav(&nav_, 0xFF); }
  Preprocessor(const Preprocessor &) = delete;
  Preprocessor &operator=(const Preprocessor &) = delete;
  // RTKLIB navigation data, filled the way the RTK node fills its own.
  nav_t &navigation() { return nav_; }
  ObservationEpoch prepare(const RawEpoch &rover,
                           const std::optional<RawEpoch> &base,
                           const Vec3 &position);

private:
  struct Receiver {
    std::map<int, double> gf;
    std::map<SF, std::tuple<std::int64_t, double, std::string, bool>>
        continuity;
    std::set<SF> seen; // bands seen with code and phase
  };
  struct Edited {
    SatelliteMotion motion;
    double elevation = 0;
    bool admitted = false;
    std::array<bool, 3> edited{true, true, true}, slip{};
  };
  Config cfg_;
  Vec3 base_;
  nav_t nav_{};
  Receiver rover_, base_receiver_;
  std::set<SF> pending_;
  std::map<int, Edited> edit(const RawEpoch &, const Vec3 &, Receiver &,
                             const std::map<int, SatelliteMotion> &,
                             int receiver_index);
  void continuity(const RawEpoch &, Receiver &, const std::map<int, Edited> &);
};
} // namespace gnss_fgo
