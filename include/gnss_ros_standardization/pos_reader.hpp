// SPDX-License-Identifier: MIT
// Shared RTKLIB .pos file parser used by pos_to_rosbag and rinex_to_rosbag.
// Supports LLH and ECEF variants, GPST date-time or week/tow time columns,
// optional velocity columns, and UTC/GPST time-system header declaration.
#pragma once

#include "gnss_ros_standardization/bag_io_utils.hpp"  // toRosTimeGpst, rtklib.h
#include <gnss_ros_standardization/msg/gnss_solution.hpp>

#include <cmath>
#include <fstream>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace gnss_pos_reader {

using GnssSolution = gnss_ros_standardization::msg::GnssSolution;

struct PosRow {
  gtime_t      t;
  GnssSolution sol;
};

struct ParseResult {
  std::vector<PosRow> rows;
  bool   is_ecef      = false;
  bool   has_velocity = false;
  size_t n_skipped    = 0;
};

namespace detail {

inline void initSolutionToNaN(GnssSolution& s) {
  const float  fnan = std::numeric_limits<float>::quiet_NaN();
  const double dnan = std::numeric_limits<double>::quiet_NaN();
  s.gdop = s.pdop = s.hdop = s.vdop = fnan;
  s.ratio    = fnan;
  s.age_diff = fnan;
  s.vel_ecef.x = s.vel_ecef.y = s.vel_ecef.z = dnan;
  s.vel_enu.x  = s.vel_enu.y  = s.vel_enu.z  = dnan;
  s.pos_enu.x  = s.pos_enu.y  = s.pos_enu.z  = dnan;
  s.pos_enu_org_ecef.x = s.pos_enu_org_ecef.y = s.pos_enu_org_ecef.z = dnan;
  for (auto& v : s.vel_cov_ecef) v = dnan;
  for (auto& v : s.vel_enu_cov)  v = dnan;
}

inline uint8_t qToStatus(int Q) {
  switch (Q) {
    case 1: return GnssSolution::STATUS_FIX;
    case 2: return GnssSolution::STATUS_FLOAT;
    case 3: return GnssSolution::STATUS_SBAS;
    case 4: return GnssSolution::STATUS_DGPS;
    case 5: return GnssSolution::STATUS_SINGLE;
    case 6: return GnssSolution::STATUS_PPP;
    default: return GnssSolution::STATUS_NONE;
  }
}

inline std::vector<std::string> splitTokens(const std::string& s) {
  std::vector<std::string> v;
  std::istringstream iss(s);
  std::string t;
  while (iss >> t) v.push_back(t);
  return v;
}

}  // namespace detail

// Parse an RTKLIB .pos file and return all valid rows.
// Errors and per-line warnings are written to `err` (typically std::cerr).
// On open failure or malformed file, an empty rows vector is returned.
inline ParseResult parsePosFile(const std::string& path, std::ostream& err) {
  ParseResult result;

  std::ifstream ifs(path);
  if (!ifs) {
    err << "[pos_reader] cannot open: " << path << "\n";
    return result;
  }

  bool is_ecef         = false;
  bool format_detected = false;
  bool has_velocity    = false;
  bool time_is_utc     = false;
  std::string line;

  // Parse header (lines starting with '%') to detect position/velocity/time format.
  std::streampos data_start = 0;
  while (true) {
    data_start = ifs.tellg();
    if (!std::getline(ifs, line)) break;
    if (line.empty()) continue;
    if (line[0] != '%') { ifs.seekg(data_start); break; }
    if (!format_detected) {
      if (line.find("x/y/z-ecef") != std::string::npos ||
          line.find("x-ecef")     != std::string::npos) {
        is_ecef = true; format_detected = true;
      } else if (line.find("lat/lon/height") != std::string::npos ||
                 line.find("latitude")       != std::string::npos) {
        is_ecef = false; format_detected = true;
      }
    }
    // RTKLIB writes "vx-ecef"/"vy-ecef"/"vz-ecef" for ECEF velocity,
    // or "ve(m/s)"/"vn(m/s)"/"vu(m/s)" for LLH velocity.
    if (line.find("vx-ecef") != std::string::npos ||
        line.find("ve(m/s)") != std::string::npos ||
        line.find("vn(m/s)") != std::string::npos) {
      has_velocity = true;
    }
    if (line.find("time sys") != std::string::npos &&
        line.find("UTC")      != std::string::npos) {
      time_is_utc = true;
    }
  }

  size_t n_warn_emitted = 0;
  const size_t kMaxWarnings = 5;
  size_t line_no = 0;
  auto warn_skip = [&](const std::string& reason) {
    ++result.n_skipped;
    if (n_warn_emitted < kMaxWarnings) {
      err << "[pos_reader] line " << line_no << " skipped: " << reason << "\n";
      if (++n_warn_emitted == kMaxWarnings)
        err << "[pos_reader] further skip warnings suppressed\n";
    }
  };

  bool   has_origin     = false;
  double origin_ecef[3] = {};
  double origin_pos[3]  = {};  // ecef2pos output: {lat_rad, lon_rad, alt_m}

  while (std::getline(ifs, line)) {
    ++line_no;
    if (line.empty() || line[0] == '%') continue;

    auto tok = detail::splitTokens(line);
    if (tok.size() < 7) { warn_skip("fewer than 7 tokens"); continue; }

    gtime_t t{};
    size_t off = 0;
    if (tok[0].find('/') != std::string::npos) {
      // "yyyy/mm/dd HH:MM:SS.sss" — str2time needs space-separated numbers.
      std::string ts = tok[0] + " " + tok[1];
      for (char& c : ts) if (c == '/' || c == ':') c = ' ';
      if (str2time(ts.c_str(), 0, (int)ts.size(), &t) != 0) {
        warn_skip("str2time failed"); continue;
      }
      off = 2;
    } else {
      try {
        int    week = std::stoi(tok[0]);
        double tow  = std::stod(tok[1]);
        t = gpst2time(week, tow);
      } catch (...) { warn_skip("week/tow parse failed"); continue; }
      off = 2;
    }
    if (time_is_utc) t = utc2gpst(t);

    if (tok.size() < off + 5) { warn_skip("missing position/Q/ns columns"); continue; }

    // Fallback format detection from first data row magnitudes.
    if (!format_detected) {
      try {
        const double a0 = std::stod(tok[off + 0]);
        const double b0 = std::stod(tok[off + 1]);
        is_ecef = !(std::fabs(a0) <= 180.0 && std::fabs(b0) <= 180.0);
        format_detected = true;
        err << "[pos_reader] format not declared in header; inferred as "
            << (is_ecef ? "ECEF" : "LLH") << "\n";
      } catch (...) { warn_skip("format inference failed"); continue; }
    }

    GnssSolution sol;
    detail::initSolutionToNaN(sol);
    rclcpp::Time stamp = gnss_converter_io::toRosTimeGpst(t);
    sol.header.stamp    = stamp;
    sol.header.frame_id = "gnss_receiver";
    int week = 0;
    sol.time_tow  = time2gpst(t, &week);
    sol.time_week = static_cast<uint32_t>(week);
    sol.solution_source = GnssSolution::SOLUTION_SOURCE_COMPUTED;

    try {
      const double a  = std::stod(tok[off + 0]);
      const double b  = std::stod(tok[off + 1]);
      const double c  = std::stod(tok[off + 2]);
      const int    Q  = std::stoi(tok[off + 3]);
      const int    ns = std::stoi(tok[off + 4]);

      sol.status   = detail::qToStatus(Q);
      sol.num_sats = static_cast<uint8_t>(std::min(255, std::max(0, ns)));

      if (is_ecef) {
        sol.pos_ecef.x = a; sol.pos_ecef.y = b; sol.pos_ecef.z = c;
        double r[3] = {a, b, c}, pos[3] = {};
        ecef2pos(r, pos);
        sol.latitude  = pos[0] * R2D;
        sol.longitude = pos[1] * R2D;
        sol.altitude  = pos[2];
      } else {
        sol.latitude = a; sol.longitude = b; sol.altitude = c;
        double pos[3] = {a * D2R, b * D2R, c}, r[3] = {};
        pos2ecef(pos, r);
        sol.pos_ecef.x = r[0]; sol.pos_ecef.y = r[1]; sol.pos_ecef.z = r[2];
      }

      // RTKLIB convention: off-diagonal sigma entries are sgn*sqrt(|cov|).
      auto sgnsq = [](double s) { return s >= 0 ? s * s : -s * s; };

      if (tok.size() >= off + 11) {
        const double s0 = std::stod(tok[off + 5]);
        const double s1 = std::stod(tok[off + 6]);
        const double s2 = std::stod(tok[off + 7]);
        const double s3 = std::stod(tok[off + 8]);
        const double s4 = std::stod(tok[off + 9]);
        const double s5 = std::stod(tok[off + 10]);
        if (is_ecef) {
          // sdx, sdy, sdz, sdxy, sdyz, sdzx
          const double cxx = s0*s0, cyy = s1*s1, czz = s2*s2;
          const double cxy = sgnsq(s3), cyz = sgnsq(s4), czx = sgnsq(s5);
          sol.pos_cov_ecef = {cxx, cxy, czx, cxy, cyy, cyz, czx, cyz, czz};
        } else {
          // sdn, sde, sdu, sdne, sdeu, sdun — stored as ENU (E,N,U) row-major.
          const double cnn = s0*s0, cee = s1*s1, cuu = s2*s2;
          const double cne = sgnsq(s3), ceu = sgnsq(s4), cun = sgnsq(s5);
          sol.pos_enu_cov = {cee, cne, ceu, cne, cnn, cun, ceu, cun, cuu};
        }
      }

      if (tok.size() >= off + 12) sol.age_diff = static_cast<float>(std::stod(tok[off + 11]));
      if (tok.size() >= off + 13) sol.ratio    = static_cast<float>(std::stod(tok[off + 12]));

      // Optional velocity block (RTKLIB --output-vel): 3 velocity + 6 sigma values.
      // sigma columns: sdvn/sdve/sdvu/sdvne/sdveu/sdvun (LLH) or sdvx/.. (ECEF).
      if (has_velocity && tok.size() >= off + 13 + 9) {
        const double va  = std::stod(tok[off + 13]);
        const double vb  = std::stod(tok[off + 14]);
        const double vc  = std::stod(tok[off + 15]);
        const double sv0 = std::stod(tok[off + 16]);
        const double sv1 = std::stod(tok[off + 17]);
        const double sv2 = std::stod(tok[off + 18]);
        const double sv3 = std::stod(tok[off + 19]);
        const double sv4 = std::stod(tok[off + 20]);
        const double sv5 = std::stod(tok[off + 21]);
        if (is_ecef) {
          sol.vel_ecef.x = va; sol.vel_ecef.y = vb; sol.vel_ecef.z = vc;
          const double vcxx = sv0*sv0, vcyy = sv1*sv1, vczz = sv2*sv2;
          const double vcxy = sgnsq(sv3), vcyz = sgnsq(sv4), vczx = sgnsq(sv5);
          sol.vel_cov_ecef = {vcxx, vcxy, vczx, vcxy, vcyy, vcyz, vczx, vcyz, vczz};
        } else {
          // RTKLIB writes vn(=va)/ve(=vb)/vu(=vc); vel_enu is E-N-U (x=E,y=N,z=U).
          sol.vel_enu.x = vb;  // E = ve
          sol.vel_enu.y = va;  // N = vn
          sol.vel_enu.z = vc;  // U = vu
          const double vcnn = sv0*sv0, vcee = sv1*sv1, vcuu = sv2*sv2;
          const double vcen = sgnsq(sv3), vceu = sgnsq(sv4), vcnu = sgnsq(sv5);
          sol.vel_enu_cov = {vcee, vcen, vceu, vcen, vcnn, vcnu, vceu, vcnu, vcuu};
        }
      }
    } catch (...) {
      warn_skip("numeric parse failed");
      continue;
    }

    // ENU position relative to the first valid position as origin.
    if (!has_origin) {
      origin_ecef[0] = sol.pos_ecef.x;
      origin_ecef[1] = sol.pos_ecef.y;
      origin_ecef[2] = sol.pos_ecef.z;
      ecef2pos(origin_ecef, origin_pos);
      has_origin = true;
    }
    sol.pos_enu_org_ecef.x = origin_ecef[0];
    sol.pos_enu_org_ecef.y = origin_ecef[1];
    sol.pos_enu_org_ecef.z = origin_ecef[2];
    {
      double dr[3]  = {sol.pos_ecef.x - origin_ecef[0],
                       sol.pos_ecef.y - origin_ecef[1],
                       sol.pos_ecef.z - origin_ecef[2]};
      double enu[3] = {};
      ecef2enu(origin_pos, dr, enu);
      sol.pos_enu.x = enu[0];
      sol.pos_enu.y = enu[1];
      sol.pos_enu.z = enu[2];
    }

    result.rows.push_back({t, sol});
  }

  result.is_ecef      = is_ecef;
  result.has_velocity = has_velocity;
  return result;
}

}  // namespace gnss_pos_reader
