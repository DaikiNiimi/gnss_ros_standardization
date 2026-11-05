#ifndef GNSS_ROS_STANDARDIZATION_RTKLIB_COMPAT_HPP
#define GNSS_ROS_STANDARDIZATION_RTKLIB_COMPAT_HPP

extern "C" {
#include "rtklib.h"
}

// RTKLIB's function-like trace macro breaks Eigen's MatrixBase::trace(), even
// when Eigen is included later by another public header. Keep backend tracing
// unchanged in C; do not leak this macro through our C++ interface.
#ifdef trace
#undef trace
#endif


// ---- rtk epoch ----
namespace gnss_utils {
// qv is an output, not the filter covariance (P/Pa). RTKLIB leaves it from
// the previous epoch when Doppler estimation fails or pntpos is skipped.
// Clear only this freshness marker: a successful current Doppler/dynamic
// estimate fills it again. Preserve rr and the complete filter state.
inline int rtkPositionEpoch(rtk_t& rtk, const obsd_t* observations,
                            int count, const nav_t& navigation) {
  for (auto& value : rtk.sol.qv) value = 0;
  return rtkpos(&rtk, observations, count, &navigation);
}
}  // namespace gnss_utils


// ---- navigation record key ----
#include <string>
#include <type_traits>

namespace gnss_utils {
namespace navigation_key_detail {
// Process-local equality key: encode fields individually, never struct padding.
// It is not a portable storage format. Exact values (including signed zeros)
// are kept, so deduplication cannot merge small but real content changes.
class Key {
 public:
  template <typename T> void add(const T& value) {
    static_assert(std::is_arithmetic_v<T>);
    bytes.append(reinterpret_cast<const char*>(&value), sizeof(value));
  }
  void add(const gtime_t& time) { add(time.time); add(time.sec); }
  template <typename T, size_t N> void add(const T (&values)[N]) {
    for (const auto& value : values) add(value);
  }
  template <typename... T> void fields(const T&... values) { (add(values), ...); }
  std::string bytes;
};
}
inline std::string navigationRecordKey(const eph_t& e) {
  navigation_key_detail::Key key;
  key.fields(e.sat,e.iode,e.iodc,e.sva,e.svh,e.week,e.code,e.flag,
             e.toe,e.toc,e.ttr,e.A,e.e,e.i0,e.OMG0,e.omg,e.M0,e.deln,e.OMGd,e.idot,
             e.crc,e.crs,e.cuc,e.cus,e.cic,e.cis,e.toes,e.fit,e.f0,e.f1,e.f2,
             e.tgd,e.Adot,e.ndot);
  return key.bytes;
}
inline std::string navigationRecordKey(const geph_t& g) {
  navigation_key_detail::Key key;
  key.fields(g.sat,g.iode,g.frq,g.svh,g.flags,g.sva,g.age,g.toe,g.tof,
             g.pos,g.vel,g.acc,g.taun,g.gamn,g.dtaun);
  return key.bytes;
}
}

#endif
