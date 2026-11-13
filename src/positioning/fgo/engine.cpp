#include "engine.hpp"
#include <Eigen/QR>
#include <Eigen/SVD>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseRotationPrior.h>
#include <algorithm>
#include <numeric>
#include <sstream>

namespace gnss_fgo {
namespace {
auto noise(int n, double sigma) {
  return gtsam::noiseModel::Isotropic::Sigma(n, sigma);
}
auto splitNoise(double a, double b) {
  gtsam::Vector6 s;
  s << a, a, a, b, b, b;
  return gtsam::noiseModel::Diagonal::Sigmas(s);
}
auto poseNoise(double a, double b, double c, double x, double y, double z) {
  gtsam::Vector6 s;
  s << a, b, c, x, y, z;
  return gtsam::noiseModel::Diagonal::Sigmas(s);
}
double median(std::vector<double> x) {
  if (x.empty())
    return 0.;
  std::sort(x.begin(), x.end());
  auto n = x.size();
  return n % 2 ? x[n / 2] : (x[n / 2 - 1] + x[n / 2]) * .5;
}
Mat3 attitude(double roll, double pitch, double heading) {
  Mat3 ned_to_enu;
  ned_to_enu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  Mat3 flu_to_frd = Vec3(1, -1, -1).asDiagonal();
  return ned_to_enu * gtsam::Rot3::RzRyRx(roll, pitch, heading).matrix() *
         flu_to_frd;
}
// Inverse of attitude(): (roll, pitch, heading) [rad] of a body (FLU) to ENU
// rotation (heading in (-pi, pi], 0 = north, positive towards east).
Vec3 rollPitchHeading(const Mat3 &R) {
  Mat3 ned_to_enu;
  ned_to_enu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  const Mat3 flu_to_frd = Vec3(1, -1, -1).asDiagonal();
  // attitude(): R = N Rz(h) Ry(p) Rx(r) F with N, F symmetric orthogonal.
  return gtsam::Rot3(ned_to_enu * R * flu_to_frd).rpy();
}
// Value of `sat` in a per-satellite residual map; 0 when it has none.
double residualOf(const std::map<int, double> &residuals, int sat) {
  const auto it = residuals.find(sat);
  return it == residuals.end() ? 0. : it->second;
}
bool geo(const SatelliteObservation &s) {
  return s.system == 3 && (s.prn <= 5 || s.prn >= 59);
}
std::map<int, std::vector<int>> groups(const ObservationEpoch &obs) {
  std::map<int, std::vector<int>> out;
  for (std::size_t i = 0; i < obs.sats.size(); ++i)
    out[obs.sats[i].system].push_back(i);
  return out;
}
Mat3 enuRotation(const Vec3 &p) {
  constexpr double e2 = 6.6943799901413165e-3, a = 6378137.;
  const double r2 = p.head<2>().squaredNorm();
  double z = p.z(), v = a;
  for (int i = 0; i < 100; ++i) {
    const double old = z;
    const double sinp = z / std::sqrt(r2 + z * z);
    v = a / std::sqrt(1 - e2 * sinp * sinp);
    z = p.z() + v * e2 * sinp;
    if (std::abs(z - old) < 1e-4)
      break;
  }
  const double lat = std::atan2(z, std::sqrt(r2)),
               lon = std::atan2(p.y(), p.x());
  const double sl = std::sin(lat), cl = std::cos(lat), sn = std::sin(lon),
               cn = std::cos(lon);
  Mat3 R;
  R << -sn, -sl * cn, cl * cn, cn, -sl * sn, cl * sn, 0, cl, sl;
  return R;
}
// 0.5 * squared whitened residual of a DD factor without its robust loss:
// the Gaussian error the residual thresholds are stated in (error() itself
// for a factor without a robust noise model).
double gaussianError(const gtsam::NonlinearFactor &f,
                     const gtsam::Values &values) {
  if (const auto *nm = dynamic_cast<const gtsam::NoiseModelFactor *>(&f))
    if (const auto robust =
            std::dynamic_pointer_cast<const gtsam::noiseModel::Robust>(
                nm->noiseModel()))
      return .5 * robust->noise()->whiten(nm->unwhitenedError(values))
                      .squaredNorm();
  return f.error(values);
}
// Ends the hold and the arc without a seed: the next key starts from code
// minus carrier with the creation prior, so an integer the data contradict is
// not carried into the new arc.
void restartArc(SatelliteState &st) {
  st.release(false);
  st.last_held.reset();
}
} // namespace
std::unique_ptr<Engine::Smoother> Engine::makeSmoother(double lag,
                                                       const Config &c,
                                                       bool qr) {
  c.validate();
  gtsam::ISAM2Params p;
  p.relinearizeSkip = c.isam2_relinearize_skip;
  p.relinearizeThreshold = c.isam2_relinearize_threshold;
  // Reuse the slots of removed and marginalized factors; otherwise the factor
  // vector grows by every factor ever added (unbounded over a long run).
  // New factors are addressed through ISAM2Result::newFactorsIndices.
  p.findUnusedFactorSlots = true;
  if (qr)
    p.factorization = gtsam::ISAM2Params::QR;
  return std::make_unique<Smoother>(lag, p);
}
Engine::Engine(const Config &c, const Vec3 &initial, const Vec3 &base,
               const Vec3 &lever)
    : cfg_(c), base_(base), lever_(lever), nav_position_(initial),
      R_(enuRotation(base)), frame_(gtsam::Rot3(R_), base) {
  if (!initial.allFinite() || !base.allFinite() || !lever.allFinite())
    throw std::invalid_argument("nonfinite initial coordinates");
  if (cfg_.gnss_motion && lever.norm() != 0)
    throw std::invalid_argument(
        "GNSS-only state is the antenna; no body lever arm");
  // Both nodes start with the motion graph; gnss_imu_fgo switches to the IMU
  // graph later (updateSwitchWindow). gnss_fgo reports phase 2 throughout.
  phase_ = cfg_.gnss_motion ? 2 : 1;
  graph_epoch_ = -1;
  actual_lag_ = cfg_.fls_lag;
  p2_ = makeSmoother(actual_lag_, cfg_, true);
}
ImuFrame Engine::imuFrame(const Vec3 &body) const {
  // With omegaCoriolis set, GTSAM's prediction adds the Coriolis term
  // -2 W x v and the centrifugal term -W x (W x p) of the position p relative
  // to the frame origin (the base). Normal gravity at the body already holds
  // the whole centrifugal acceleration there, which is that of the base plus
  // -W x (W x p): the frame gravity is normal gravity plus W x (W x p), i.e.
  // gravitation at the body and the centrifugal acceleration of the base, so
  // that no term is counted twice.
  const Vec3 w = earthRate(R_);
  return {localGravity(base_ + R_ * body, R_) + w.cross(w.cross(body)), w};
}
double Engine::gravityAt(const Vec3 &ecef) const {
  const Vec3 g = geodeticWgs84(ecef);
  return normalGravity(g.x(), g.z());
}
Mat3 Engine::localToBase(const Vec3 &ecef) const {
  return R_.transpose() * enuRotation(ecef);
}
void Engine::addImu(const ImuSample &s) {
  if (cfg_.gnss_motion)
    throw std::invalid_argument(
        "IMU input is not part of the GNSS-only model");
  if (!std::isfinite(s.gpst) || (!imu_.empty() && s.gpst <= imu_.back().gpst))
    throw std::invalid_argument("unordered IMU input");
  auto sample = s;
  if (!imu_.empty())
    sample.predecessor = imu_.back().gpst;
  imu_.push_back(sample);
  // Bounded backlog: without GNSS epochs nothing consumes IMU samples. Once
  // more than kImuBacklog + kImuBacklogBlock wait, drop the oldest unconsumed
  // ones down to kImuBacklog (counted), so the erase runs once per block. The
  // sample after the dropped ones keeps the last dropped sample as its
  // predecessor; the gap fails the next interval check, which resets the
  // motion state instead of integrating across it.
  const std::size_t waiting = imu_.size() - imu_index_;
  if (waiting > kImuBacklog + kImuBacklogBlock) {
    const std::size_t drop = waiting - kImuBacklog;
    const auto first = imu_.begin() + imu_index_;
    imu_.erase(first, first + drop);
    imu_backlog_dropped_ += drop;
  }
}
void Engine::pruneImu() {
  // Consumed samples are needed only for the initialization statistics (the
  // first kInitSamples, while still in the motion stage) and the last consumed
  // sample (output angular rate). The switch window keeps its own copies.
  // Erase the rest in batches.
  constexpr std::size_t kInitSamples = 3000, kBatch = 4096;
  if (imu_index_ == 0)
    return;
  const std::size_t head =
      phase_ == 1 ? std::min(kInitSamples, imu_index_) : 0;
  const std::size_t tail = imu_index_ - 1;
  if (tail < head + kBatch)
    return;
  imu_.erase(imu_.begin() + head, imu_.begin() + tail);
  imu_index_ -= tail - head;
}
void Engine::slips(const ObservationEpoch &obs) {
  auto reset = obs.input_slips;
  std::set<SF> visible;
  for (const auto &s : obs.sats) {
    for (int f = 0; f < 3; ++f) {
      SF sf{s.sat, f};
      auto &st = sats_.get(sf);
      if (s.phase[f] != 0 && s.phase_base[f] != 0) {
        visible.insert(sf);
        st.outage = 0;
      }
      if (s.lli[f] & 1)
        reset.insert(sf);
    }
    if (s.phase[0] && s.phase[1] && s.wavelength[0] > 0 &&
        s.wavelength[1] > 0) {
      double gf = s.phase[0] * s.wavelength[0] - s.phase[1] * s.wavelength[1];
      auto it = sats_.geometry_free.find(s.sat);
      if (it != sats_.geometry_free.end() && it->second != 0 &&
          std::abs(gf - it->second) > cfg_.thres_slip)
        for (int f = 0; f < 3; ++f)
          reset.insert({s.sat, f});
      sats_.geometry_free[s.sat] = gf;
    }
  }
  for (auto &[sf, st] : sats_.track)
    if (!visible.count(sf) && ++st.outage == 6)
      reset.insert(sf);
  for (auto sf : reset)
    sats_.get(sf).invalidate();
}
int Engine::reference(const ObservationEpoch &obs,
                      const std::vector<int> &idx) {
  const int sys = obs.sats[idx.front()].system;
  // current_.references: buildDd's choice earlier in this epoch (EpochState is
  // reset when an epoch starts or fallback() re-solves it). Later calls in the
  // epoch (transition()'s buildDd per window epoch, ddprPosition after buildDd)
  // keep it while usable; a new epoch chooses anew by residual and elevation.
  auto prev = current_.references.find(sys);
  for (int i : idx)
    if (prev != current_.references.end() && obs.sats[i].sat == prev->second &&
        !geo(obs.sats[i]) &&
        residualOf(per_sat_res_, prev->second) <= cfg_.per_sat_res_thresh)
      return i;
  std::vector<int> pool;
  for (int i : idx)
    if (!geo(obs.sats[i]) &&
        residualOf(per_sat_res_, obs.sats[i].sat) <=
            cfg_.per_sat_res_thresh)
      pool.push_back(i);
  if (pool.empty())
    for (int i : idx)
      if (!geo(obs.sats[i]))
        pool.push_back(i);
  if (pool.empty())
    pool = idx;
  return *std::max_element(pool.begin(), pool.end(), [&](int i, int j) {
    return obs.sats[i].elevation < obs.sats[j].elevation;
  });
}
std::vector<std::array<double, 2>>
Engine::slantDelays(const ObservationEpoch &obs, const Vec3 &rover) const {
  std::vector<std::array<double, 2>> out(obs.sats.size());
  for (std::size_t i = 0; i < obs.sats.size(); ++i)
    out[i] = {hydrostaticDelay(rover, obs.sats[i].rover_sat, obs.gpst),
              hydrostaticDelay(base_, obs.sats[i].base_sat, obs.base_gpst)};
  return out;
}
int Engine::buildDd(Graph &graph, DdRows &rows, Values &values,
                    const ObservationEpoch &obs, Key pose, const Vec3 &lever) {
  current_.visible.clear();
  int nv = 0;
  // For the gauge prior below: the keys created in this call.
  std::set<Key> created;
  // Slant delays at the antenna of the pose's initial value in this update
  // (the linearization point; nav_position_ when the pose has none here).
  const auto delay = slantDelays(
      obs, values.exists(pose) ? antenna(values.at<gtsam::Pose3>(pose), lever)
                               : nav_position_);
  for (const auto &[sys, idx] : groups(obs)) {
    if (idx.size() < 2)
      continue;
    const int ri = reference(obs, idx);
    const auto &r = obs.sats[ri];
    const auto &dr = delay[ri];
    current_.references[sys] = r.sat;
    // Per band: carrier DDs added, one of their keys existed before this
    // call, one of their keys was seeded from a held value.
    std::array<bool, 3> carrier{}, continuing{}, seeded{};
    for (int ji : idx) {
      if (ji == ri)
        continue;
      const auto &j = obs.sats[ji];
      const auto &dj = delay[ji];
      if (cfg_.exclude_bds_geo && geo(j))
        continue;
      for (int f = 0; f < 3; ++f) {
        if (!r.pr[f] || !r.pr_base[f] || !j.pr[f] || !j.pr_base[f])
          continue;
        // Pseudoranges [m] minus the slant delays; error = predicted DD range
        // - measured DD.
        const double pr_rov_ref = r.pr[f] - dr[0],
                     pr_base_ref = r.pr_base[f] - dr[1],
                     pr_rov_target = j.pr[f] - dj[0],
                     pr_base_target = j.pr_base[f] - dj[1];
        rows.push_back({graph.size(), false, r.sat, j.sat, f});
        graph.emplace_shared<gtsam::DoubleDifferencePseudorangeFactorArm>(
            pose, pr_rov_ref, pr_base_ref, pr_rov_target, pr_base_target,
            r.rover_sat, j.rover_sat, r.base_sat, j.base_sat, base_, lever,
            frame_,
            gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Huber::Create(kCodeHuber),
                gtsam::noiseModel::Isotropic::Sigma(
                    1, ddSigma(cfg_, true, r.elevation, j.elevation,
                               obs.gpst - obs.base_gpst))));
        ++nv;
        const double lam = r.wavelength[f];
        if (sys == 4 || lam <= 0 || !r.phase[f] ||
            !r.phase_base[f] || !j.phase[f] || !j.phase_base[f])
          continue;
        current_.visible.insert({r.sat, f});
        current_.visible.insert({j.sat, f});
        for (const auto *sat : {&r, &j}) {
          SF sf{sat->sat, f};
          auto &st = sats_.get(sf);
          if (st.key)
            continue;
          // New arc: a new key with its only prior.
          Key key = ambKey(sf, st.generation);
          double n = ((sat->phase[f] * lam - sat->phase_base[f] * lam) -
                      (sat->pr[f] - sat->pr_base[f])) /
                     lam;
          double sigma = cfg_.sigma_amb0;
          if (st.seed_pending && st.last_held) {
            n = *st.last_held;
            sigma = .1;
            current_.consumed.insert(sf);
            seeded[f] = true;
          }
          current_.seeded.insert(sf);
          if (!values.exists(key)) {
            values.insert(key, n);
            graph.addPrior<double>(key, n, noise(1, sigma));
          }
          arc_created_[key] = gnss_epochs_;
          st.key = key;
          st.system = sys;
          created.insert(key);
        }
        carrier[f] = true;
        for (int sat : {r.sat, j.sat})
          if (!created.count(*sats_.get({sat, f}).key))
            continuing[f] = true;
        const double cp_sigma = ddSigma(cfg_, false, r.elevation, j.elevation,
                                        obs.gpst - obs.base_gpst);
        rows.push_back({graph.size(), true, r.sat, j.sat, f});
        // Carrier phases in metres (cycles * lam) minus the slant delays;
        // ambiguities in cycles; error = predicted DD range + lam (N_ref -
        // N_target) - DD. A held ambiguity keeps its key; the hold is a
        // separate factor (hold()).
        const double cp_rov_ref = r.phase[f] * lam - dr[0],
                     cp_base_ref = r.phase_base[f] * lam - dr[1],
                     cp_rov_target = j.phase[f] * lam - dj[0],
                     cp_base_target = j.phase_base[f] * lam - dj[1];
        graph.emplace_shared<gtsam::DoubleDifferenceCarrierPhaseFactorArm>(
            pose, *sats_.get({r.sat, f}).key, *sats_.get({j.sat, f}).key,
            cp_rov_ref, cp_base_ref, cp_rov_target, cp_base_target,
            r.rover_sat, j.rover_sat, r.base_sat, j.base_sat, base_, lam,
            lever, frame_, gtsam::noiseModel::Isotropic::Sigma(1, cp_sigma));
        ++nv;
      }
    }
    // Gauge: carrier DDs and holds see only differences of a group's
    // ambiguities, so its common mode is held only by the weak creation
    // priors and turns numerically indeterminate as the difference
    // information grows (ISAM2 Cholesky). One tight prior on the reference key
    // at its initial value fixes it; the position and the differences do not
    // depend on the common mode (apart from the weak pull of the creation
    // priors). Only where the group starts: a continuing key carries the
    // gauge (also through marginalization) and a seed prior already fixes it.
    if (ambiguity_gauge_)
      for (int f = 0; f < 3; ++f)
        if (carrier[f] && !continuing[f] && !seeded[f]) {
          const Key ref = *sats_.get({r.sat, f}).key;
          graph.addPrior<double>(ref, values.at<double>(ref),
                                 noise(1, kAmbiguityGaugeSigma));
        }
  }
  return nv;
}
DdPosition Engine::ddprPosition(const ObservationEpoch &obs,
                                const gtsam::Pose3 &initial,
                                const Vec3 &lever) {
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
  const Key key = gtsam::Symbol('z', 0);
  // Slant delays at the antenna of `initial` (as in buildDd).
  const auto delay = slantDelays(obs, antenna(initial, lever));
  for (const auto &[sys, idx] : groups(obs)) {
    if (idx.size() < 2)
      continue;
    int ri = reference(obs, idx);
    const auto &r = obs.sats[ri];
    const auto &dr = delay[ri];
    for (int ji : idx) {
      if (ji == ri)
        continue;
      const auto &j = obs.sats[ji];
      const auto &dj = delay[ji];
      for (int f = 0; f < 3; ++f) {
        if (!r.pr[f] || !r.pr_base[f] || !j.pr[f] || !j.pr_base[f])
          continue;
        factors.push_back(
            std::make_shared<gtsam::DoubleDifferencePseudorangeFactorArm>(
                key, r.pr[f] - dr[0], r.pr_base[f] - dr[1], j.pr[f] - dj[0],
                j.pr_base[f] - dj[1], r.rover_sat, j.rover_sat, r.base_sat,
                j.base_sat, base_, lever, frame_,
                gtsam::noiseModel::Isotropic::Sigma(
                    1, cfg_.sigma_pr * std::sqrt(2.))));
      }
    }
  }
  DdPosition out;
  if (factors.size() < 4)
    return out;
  std::vector<int> active(factors.size());
  std::iota(active.begin(), active.end(), 0);
  for (int pass = 0; pass < 4; ++pass) {
    Graph graph;
    Values values;
    values.insert(key, initial);
    graph.addPrior<gtsam::Pose3>(key, initial,
                                 poseNoise(.05, .05, .1, 50, 50, 50));
    for (int i : active)
      graph.push_back(factors[i]);
    gtsam::LevenbergMarquardtParams params;
    params.maxIterations = 10;
    Values estimate =
        gtsam::LevenbergMarquardtOptimizer(graph, values, params).optimize();
    std::vector<int> keep;
    double sum = 0;
    int dropped = 0;
    for (int i : active) {
      double res = std::sqrt(2. * std::max(0., factors[i]->error(estimate))) *
                   cfg_.sigma_pr * std::sqrt(2.);
      if (pass < 3 && res > cfg_.fde_pr && int(active.size()) - dropped > 4) {
        ++dropped;
        continue;
      }
      keep.push_back(i);
      sum += res * res;
    }
    if (!dropped) {
      out.position = R_ * estimate.at<gtsam::Pose3>(key).translation() + base_;
      out.active = active.size();
      out.rms = std::sqrt(sum / keep.size());
      return out;
    }
    active = std::move(keep);
  }
  return out;
}
Engine::Values Engine::fde(Smoother &sm, const Graph &graph, const DdRows &rows,
                           const gtsam::FactorIndices &slots, Values est,
                           Key pose) {
  if (slots.size() != graph.size())
    throw std::logic_error("factor slots do not match the update graph");
  for (const auto &row : rows)
    if (row.index >= graph.size() || !graph[row.index])
      throw std::logic_error("DD rows do not match the update graph");
  if (!cfg_.fde_enable)
    return est;
  // Carrier DD rows above fde_cp are removed; when more than fde_max_frac of
  // them are, the estimate itself is suspect and nothing is removed.
  gtsam::FactorIndices rejected;
  int carrier = 0;
  for (const auto &row : rows) {
    if (!row.carrier)
      continue;
    ++carrier;
    const double res = std::sqrt(2 * graph[row.index]->error(est)) *
                       cfg_.sigma_cp * std::sqrt(2.);
    if (res > cfg_.fde_cp)
      rejected.push_back(slots[row.index]);
  }
  if (rejected.empty() ||
      rejected.size() > cfg_.fde_max_frac * std::max(1, carrier))
    return est;
  try {
    updateProbe("fde", false);
    sm.update(Graph{}, Values{}, gtsam::FixedLagSmoother::KeyTimestampMap{},
              rejected);
    updateProbe("fde", true);
    est = sm.calculateEstimate();
    if (!est.exists(pose) || !est.at<gtsam::Pose3>(pose).matrix().allFinite())
      throw std::runtime_error("FDE lost pose");
  } catch (...) {
    healthy_ = false;
    throw;
  }
  std::set<std::size_t> removed(rejected.begin(), rejected.end());
  current_.visible.clear();
  for (const auto &row : rows) {
    if (!row.carrier)
      continue;
    if (removed.count(slots[row.index])) {
      // A rejected carrier DD releases the holds it contradicts.
      for (int sat : {row.ref_sat, row.target_sat}) {
        auto &st = sats_.get({sat, row.band});
        if (st.held)
          restartArc(st);
      }
    } else {
      current_.visible.insert({row.ref_sat, row.band});
      current_.visible.insert({row.target_sat, row.band});
    }
  }
  return est;
}
Engine::ArSearch
Engine::arSearch(const std::vector<SF> &keys,
                 const std::map<int, const SatelliteObservation *> &present,
                 const std::map<SF, int> &index, const Eigen::VectorXd &x,
                 const Eigen::MatrixXd &cov,
                 const std::vector<char> *mask) const {
  ArSearch s;
  const int n = keys.size();
  std::map<std::pair<int, int>, std::vector<SF>> grouped;
  for (int i = 0; i < n; ++i)
    if (!mask || (*mask)[i]) {
      const auto sf = keys[i];
      grouped[{present.at(sf.first)->system, sf.second}].push_back(sf);
    }
  auto &pairs = s.pairs;
  for (auto &[group, members] : grouped) {
    std::sort(members.begin(), members.end());
    for (std::size_t i = 1; i < members.size(); ++i)
      pairs.push_back({members[0], members[i]});
  }
  if (pairs.size() < 2)
    return s;
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(pairs.size(), n);
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    D(i, index.at(pairs[i].first)) = 1;
    D(i, index.at(pairs[i].second)) = -1;
  }
  Eigen::VectorXd y = D * x;
  Eigen::MatrixXd Q = D * cov * D.transpose();
  s.res = gnss_fgo::ar::resolve(y, Q, 2, cfg_.ar_thresar);
  s.D = std::move(D);
  s.Q = std::move(Q);
  s.y = std::move(y);
  return s;
}
bool Engine::stationary(const ObservationEpoch &obs, const Values &estimate,
                        Key pose) {
  const Key vel_key = velKey(int(gtsam::Symbol(pose).index()));
  const bool has_vel = estimate.exists(vel_key);
  std::optional<double> speed;
  std::optional<DopplerRows> local;
  const DopplerRows *rows = epoch_doppler_ ? &*epoch_doppler_ : nullptr;
  if (!rows) {
    local = dopplerRows(
        obs, gtsam::NavState(estimate.at<gtsam::Pose3>(pose),
                             has_vel ? estimate.at<Vec3>(vel_key)
                                     : Vec3(Vec3::Zero())));
    if (local)
      rows = &*local;
  }
  if (rows)
    if (const auto v = dopplerVelocity(*rows))
      speed = v->norm();
  if (!speed && has_vel) {
    const double s = estimate.at<Vec3>(vel_key).norm();
    if (std::isfinite(s))
      speed = s;
  }
  return speed && *speed < kStationarySpeed;
}
bool Engine::heldIntegersAgree(const std::vector<std::pair<SF, SF>> &pairs,
                               const Eigen::VectorXd &integers) const {
  // Each hold record (kr, kj) with difference d says N_kj - N_kr = d.
  // Potentials are assigned per connected component of the record graph by a
  // breadth-first walk from the component's first key.
  std::map<Key, std::vector<std::pair<Key, double>>> adjacent;
  for (const auto &[pair, h] : holds_) {
    adjacent[pair.first].push_back({pair.second, h.difference});
    adjacent[pair.second].push_back({pair.first, -h.difference});
  }
  std::map<Key, std::pair<Key, double>> potential; // key -> (root, potential)
  auto walk = [&](Key start) {
    if (potential.count(start) || !adjacent.count(start))
      return;
    std::vector<Key> queue{start};
    potential[start] = {start, 0.};
    for (std::size_t q = 0; q < queue.size(); ++q) {
      const Key key = queue[q];
      const double base = potential[key].second;
      for (const auto &[next, d] : adjacent[key])
        if (!potential.count(next)) {
          potential[next] = {start, base + d};
          queue.push_back(next);
        }
    }
  };
  // Pair i = (r, j) fixes N_r - N_j = integers[i]: it agrees when both keys
  // are in one component and -integers[i] is their potential difference.
  std::set<Key> roots;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const auto &kr = sats_.track.at(pairs[i].first).key,
               &kj = sats_.track.at(pairs[i].second).key;
    if (!kr || !kj)
      return false;
    walk(*kr);
    walk(*kj);
    const auto ir = potential.find(*kr), ij = potential.find(*kj);
    if (ir == potential.end() || ij == potential.end() ||
        ir->second.first != ij->second.first)
      return false;
    roots.insert(ir->second.first);
    const double held = ij->second.second - ir->second.second;
    if (!(std::abs(-integers[i] - held) < .5))
      return false;
  }
  // Every record of the components involved must fit the potentials, else
  // the held relations contradict each other.
  for (const auto &[pair, h] : holds_) {
    const auto ia = potential.find(pair.first);
    if (ia == potential.end() || !roots.count(ia->second.first))
      continue;
    const double p = potential.at(pair.second).second - ia->second.second;
    if (!(std::abs(p - h.difference) < .5))
      return false;
  }
  return true;
}
std::vector<SF> Engine::searchArcs(const ObservationEpoch &obs,
                                   const Values &estimate) const {
  std::set<int> present;
  for (const auto &s : obs.sats)
    present.insert(s.sat);
  std::vector<SF> arcs;
  for (auto [sf, key] : sats_.keys())
    if (present.count(sf.first) && current_.visible.count(sf) &&
        !current_.seeded.count(sf) && estimate.exists(key))
      arcs.push_back(sf);
  return arcs;
}
int Engine::heldArcs(const std::vector<SF> &arcs) const {
  int held = 0;
  for (auto sf : arcs)
    held += sats_.track.at(sf).held.has_value();
  return held;
}
ArDecision Engine::resolve(Smoother &sm, const Values &estimate, Key pose,
                           const ObservationEpoch &obs) {
  std::map<int, const SatelliteObservation *> present;
  for (const auto &s : obs.sats)
    present[s.sat] = &s;
  // Every eligible ambiguity, held or not, enters with its estimate and its
  // joint marginal covariance (a hold is a factor on the key, not a constant).
  const std::vector<SF> keys = searchArcs(obs, estimate);
  std::map<SF, double> floats;
  std::map<SF, Key> amb;
  for (auto sf : keys) {
    amb[sf] = *sats_.track.at(sf).key;
    floats[sf] = estimate.at<double>(amb[sf]);
  }
  ArDecision out;
  if (keys.size() < 2)
    return out;
  // Fewest fixed pairs of the full set and the partial retry: stricter at
  // rest while few of the eligible arcs are held.
  const int held = heldArcs(keys);
  const int min_pairs = held < kFewHeldArcs && stationary(obs, estimate, pose)
                            ? kStationaryMinFixPairs
                            : kMinFixPairs;
  gtsam::KeyVector query{pose};
  for (auto sf : keys)
    query.push_back(amb[sf]);
  const gtsam::JointMarginal jm = sm.getISAM2().jointMarginalCovariance(query);
  int n = keys.size();
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(n, n),
                  cross = Eigen::MatrixXd::Zero(3, n);
  Eigen::VectorXd x(n);
  std::map<SF, int> index;
  const Mat3 rot = estimate.at<gtsam::Pose3>(pose).rotation().matrix();
  const Vec3 lever = stageLever();
  for (int i = 0; i < n; ++i) {
    auto sf = keys[i];
    index[sf] = i;
    x[i] = floats[sf];
    for (int j = 0; j < n; ++j)
      cov(i, j) = jm.at(amb[sf], amb[keys[j]])(0, 0);
    const gtsam::Vector6 block = jm.at(pose, amb[sf]).col(0);
    cross.col(i) = rot * (block.tail<3>() - lever.cross(block.head<3>()));
  }
  // The decision of an accepted search `s` into `d`: its fixed values and
  // pairs, the conditional antenna position and covariance and nb. False
  // (refused) with fewer than ar_minfixsats satellites or `min` pairs.
  const auto fix = [&](const ArSearch &s, int min, ArDecision &d) {
    std::set<int> used;
    for (std::size_t i = 0; i < s.pairs.size(); ++i) {
      auto [r, j] = s.pairs[i];
      used.insert(r.first);
      used.insert(j.first);
      d.fixed.emplace(r, floats.at(r));
      d.fixed[j] = d.fixed[r] - s.res.integers[i];
      d.allowed.insert(r);
      d.allowed.insert(j);
      d.fixed_pairs.push_back(s.pairs[i]);
    }
    if (int(used.size()) < cfg_.ar_minfixsats || s.res.kept < min)
      return false;
    Eigen::VectorXd fixed = x;
    for (int i = 0; i < n; ++i)
      if (d.fixed.count(keys[i]))
        fixed[i] = d.fixed[keys[i]];
    Eigen::LLT<Eigen::MatrixXd> chol(s.Q);
    if (chol.info() != Eigen::Success)
      throw std::runtime_error("invalid conditional covariance");
    const Eigen::Vector3d correction =
        cross * s.D.transpose() * s.Q.partialPivLu().solve(s.D * (x - fixed));
    d.fixed_position = nav_position_ - R_ * correction;
    if (!d.fixed_position.allFinite())
      throw std::runtime_error("nonfinite conditional position");
    // Antenna position covariance conditioned on the fixed D x = integers:
    // P - C Q^-1 C', with C = cross D'.
    // Output only; it does not enter the estimate.
    const gtsam::Matrix6 P6 = jm.at(pose, pose);
    Eigen::Matrix<double, 3, 6> J;
    J.leftCols<3>() = -rot * gtsam::skewSymmetric(lever);
    J.rightCols<3>() = rot;
    const Mat3 P = J * P6 * J.transpose();
    const Eigen::MatrixXd C = cross * s.D.transpose();
    Mat3 conditioned = P - C * chol.solve(C.transpose());
    conditioned = (.5 * (conditioned + conditioned.transpose())).eval();
    if (conditioned.allFinite() && (conditioned.diagonal().array() >= 0).all())
      d.fixed_covariance = conditioned;
    d.nb = s.res.kept;
    return true;
  };
  // A retry on the arcs `mask` selects: the search, the ratio test (same
  // threshold), for the held set the agreement with the holds, then fix().
  const auto retry = [&](const std::vector<char> &mask, int min,
                         bool held_only) -> std::optional<ArDecision> {
    const ArSearch s = arSearch(keys, present, index, x, cov, &mask);
    if (s.pairs.size() < 2)
      return std::nullopt;
    if (s.res.error)
      throw std::runtime_error(
          "invalid integer-search covariance or reduction");
    if (!s.res.accepted ||
        (held_only && !heldIntegersAgree(s.pairs, s.res.integers)))
      return std::nullopt;
    ArDecision d;
    d.s0 = s.res.s0;
    d.s1 = s.res.s1;
    if (s.res.s0 <= 1e-12 * std::max(s.res.s1, 1.))
      d.s0 = 0;
    if (!fix(s, min, d))
      return std::nullopt;
    d.held_only = held_only;
    return d;
  };
  const ArSearch search = arSearch(keys, present, index, x, cov, nullptr);
  if (search.pairs.size() < 2) {
    if (search.pairs.size() == 1)
      ar_outcome_ = ArOutcome::MinPairs;
    return out;
  }
  out.s0 = search.res.s0;
  out.s1 = search.res.s1;
  if (search.res.error)
    throw std::runtime_error("invalid integer-search covariance or reduction");
  if (search.res.s0 <= 1e-12 * std::max(search.res.s1, 1.))
    out.s0 = 0;
  if (search.res.accepted) {
    if (fix(search, min_pairs, out))
      return out;
    // Refused by the minimum rules: no ratio is reported.
    out = ArDecision{};
  }
  // Partial retry: the held arcs and those old enough for their float value
  // to have converged; an immature arc no longer blocks the mature set.
  std::vector<char> mature(n), held_set(n);
  int masked = 0;
  for (int i = 0; i < n; ++i) {
    held_set[i] = sats_.track.at(keys[i]).held.has_value();
    mature[i] = held_set[i] || arcAge(amb[keys[i]]) >= kMatureArcEpochs;
    masked += mature[i];
  }
  if (masked < n)
    if (auto d = retry(mature, min_pairs, false))
      return *d;
  // The held arcs alone, accepted only when they reproduce the hold
  // integers: confirms the held set while new arcs disturb the full search.
  if (auto d = retry(held_set, kHeldSetMinPairs, true))
    return *d;
  return out;
}
ArDecision Engine::runAr(Smoother &sm, const Values &estimate, Key pose,
                         const ObservationEpoch &obs) {
  ArDecision result;
  ar_outcome_ = ArOutcome::NotCalled;
  if (!cfg_.ar_mode)
    return result;
  try {
    result = resolve(sm, estimate, pose, obs);
  } catch (const std::exception &) {
    return {};
  }
  // A held-only fix changes no state: bookkept like an epoch without a fix.
  if (result.nb && !result.held_only) {
    for (auto sf : result.allowed)
      current_.accepted[sf] = sats_.get(sf).arc;
  } else if (ar_outcome_ != ArOutcome::MinPairs)
    ar_outcome_ = ArOutcome::Zero;
  return result;
}
void Engine::hold(Smoother &sm, const ArDecision &ar) {
  if (!ar.nb)
    return;
  // Soft constraint N_r - N_j = fixed[r] - fixed[j] on each accepted pair
  // (BetweenFactor error: (N_j - N_r) - measured). The keys last for their
  // arc, so a pair of keys carries at most one hold factor: a pair already
  // held at the same integer gets nothing new, a changed integer replaces the
  // factor (removed and added in one update).
  std::set<Key> active;
  for (auto [sf, key] : sats_.keys())
    active.insert(key);
  // A key whose arc ended is no longer refreshed; its hold factor stays until
  // the key leaves through the lag, and its record is dropped now.
  for (auto it = holds_.begin(); it != holds_.end();)
    if (!active.count(it->first.first) || !active.count(it->first.second))
      it = holds_.erase(it);
    else
      ++it;
  Graph graph;
  gtsam::FactorIndices remove;
  std::vector<std::pair<std::pair<Key, Key>, double>> added;
  for (const auto &[r, j] : ar.fixed_pairs) {
    const auto &kr = sats_.get(r).key, &kj = sats_.get(j).key;
    if (!kr || !kj)
      continue;
    const std::pair<Key, Key> pair{*kr, *kj};
    const double difference = ar.fixed.at(j) - ar.fixed.at(r);
    if (auto it = holds_.find(pair); it != holds_.end()) {
      if (std::abs(it->second.difference - difference) < .5)
        continue;
      const auto &factors = sm.getFactors();
      const auto *previous =
          it->second.slot < factors.size()
              ? dynamic_cast<const gtsam::BetweenFactor<double> *>(
                    factors[it->second.slot].get())
              : nullptr;
      if (!previous ||
          previous->keys() != gtsam::KeyVector{pair.first, pair.second})
        throw std::logic_error("hold record does not match its factor");
      remove.push_back(it->second.slot);
    }
    graph.emplace_shared<gtsam::BetweenFactor<double>>(
        *kr, *kj, difference, noise(1, std::sqrt(cfg_.varholdamb)));
    added.push_back({pair, difference});
    // No timestamps: the smoother keeps those the epoch update gave these
    // keys, so the hold neither extends nor shortens them.
  }
  if (!graph.empty()) {
    gtsam::FactorIndices slots;
    try {
      updateProbe("hold", false);
      sm.update(graph, Values{}, gtsam::FixedLagSmoother::KeyTimestampMap{},
                remove);
      updateProbe("hold", true);
      slots = sm.getISAM2Result().newFactorsIndices;
      if (slots.size() != graph.size())
        throw std::logic_error("hold factor slots do not match the update");
    } catch (...) {
      healthy_ = false;
      throw;
    }
    for (std::size_t i = 0; i < added.size(); ++i)
      holds_[added[i].first] = {added[i].second, slots[i]};
  }
  for (auto sf : ar.allowed) {
    auto &st = sats_.get(sf);
    // Bookkeeping only: `held` is not a constant in any factor.
    if (st.key)
      st.activate(ar.fixed.at(sf));
    st.fix_flag = 3;
  }
}
Engine::Values Engine::rejectHolds(Smoother &sm, Values est, Key pose) {
  // Records whose keys the estimate lacks (left the smoother; hold() drops
  // them lazily) are skipped.
  gtsam::FactorIndices remove;
  std::vector<std::pair<Key, Key>> rejected;
  const auto &factors = sm.getFactors();
  for (const auto &[pair, h] : holds_) {
    if (!est.exists(pair.first) || !est.exists(pair.second))
      continue;
    const double r = std::abs(
        (est.at<double>(pair.second) - est.at<double>(pair.first)) -
        h.difference);
    if (!(r > kHoldRejectCycles))
      continue;
    const auto *factor =
        h.slot < factors.size()
            ? dynamic_cast<const gtsam::BetweenFactor<double> *>(
                  factors[h.slot].get())
            : nullptr;
    if (!factor || factor->keys() != gtsam::KeyVector{pair.first, pair.second})
      throw std::logic_error("hold record does not match its factor");
    remove.push_back(h.slot);
    rejected.push_back(pair);
  }
  if (remove.empty())
    return est;
  try {
    updateProbe("hold_reject", false);
    sm.update(Graph{}, Values{}, gtsam::FixedLagSmoother::KeyTimestampMap{},
              remove);
    updateProbe("hold_reject", true);
    est = sm.calculateEstimate();
    if (!est.exists(pose) || !est.at<gtsam::Pose3>(pose).matrix().allFinite())
      throw std::runtime_error("hold rejection lost pose");
  } catch (...) {
    healthy_ = false;
    throw;
  }
  for (const auto &pair : rejected)
    holds_.erase(pair);
  // `held` = the satellite/band takes part in a hold record: a key of a
  // rejected record that is in no remaining record loses its hold (and
  // last_held, so no later path seeds from it); key and arc stay.
  std::set<Key> touched, remaining;
  for (const auto &pair : rejected)
    touched.insert({pair.first, pair.second});
  for (const auto &[pair, h] : holds_)
    remaining.insert({pair.first, pair.second});
  for (auto &[sf, st] : sats_.track)
    if (st.held && st.key && touched.count(*st.key) &&
        !remaining.count(*st.key))
      st.clearHold();
  return est;
}
void Engine::restartFloatArcs() {
  // Held: the held arcs of this epoch's search (epoch_held_arcs_, the count
  // of the stationary fix minimum); un-held carrier arcs: live keys without
  // the flag.
  int unheld = 0;
  for (const auto &[sf, st] : sats_.track)
    if (st.key && !st.held)
      ++unheld;
  if (epoch_held_arcs_ >= kFewHeldArcs || *epoch_hold_fix_)
    fixless_epochs_ = 0;
  else if (unheld > 0)
    ++fixless_epochs_;
  if (fixless_epochs_ < kFloatRestartEpochs)
    return;
  // The next carrier DD of each starts a new key from code minus carrier;
  // the old keys get no further factors and leave through the lag.
  for (auto &[sf, st] : sats_.track)
    if (st.key && !st.held)
      restartArc(st);
  fixless_epochs_ = 0;
}
Solution Engine::finalize(Solution out) {
  out.gpst = current_gpst_;
  out.phase = phase_;
  out.valid = out.valid && healthy_ && out.position.allFinite();
  if (out.valid && !motionStage()) {
    const auto estimate = p2_->calculateEstimate();
    const auto pose = estimate.at<gtsam::Pose3>(poseKey(graph_epoch_));
    out.body_to_ecef = R_ * pose.rotation().matrix();
    out.body_position = out.position - out.body_to_ecef * lever_;
    out.body_velocity = R_ * estimate.at<Vec3>(velKey(graph_epoch_));
    if (imu_index_ > 0)
      out.angular_velocity = imu_[imu_index_ - 1].gyro - bias_.gyroscope();
  }
  if (out.valid)
    fillCovariance(out);
  if (!out.valid) {
    out.kind = UpdateKind::None;
    out.fixed = false;
    out.nb = 0;
    out.position.setConstant(std::numeric_limits<double>::quiet_NaN());
  }
  // A held-only fix gives the switch window and the fix streaks the float
  // epoch (float position, marginal covariance, not fixed).
  const bool held_only = out.fixed && held_only_float_.has_value();
  if (!cfg_.gnss_motion && phase_ == 1) {
    if (held_only) {
      Solution floating = out;
      floating.position = *held_only_float_;
      floating.position_covariance.setConstant(
          std::numeric_limits<double>::quiet_NaN());
      floating.fixed = false;
      floating.nb = 0;
      floating.ratio = 0;
      floating.valid = floating.valid && floating.position.allFinite();
      if (floating.valid)
        fillCovariance(floating);
      updateSwitchWindow(floating);
    } else
      updateSwitchWindow(out);
  }
  current_.finalize(sats_, out.fixed && !held_only);
  ++input_epoch_;
  pruneImu();
  // Arc ages are kept for the live keys only.
  std::set<Key> live;
  for (const auto &[sf, key] : sats_.keys())
    live.insert(key);
  for (auto it = arc_created_.begin(); it != arc_created_.end();)
    it = live.count(it->first) ? std::next(it) : arc_created_.erase(it);
  return out;
}
void Engine::fillCovariance(Solution &out) const {
  // Output only: marginals of the current state. A FIX keeps the covariance
  // conditioned on its integers, set where the FIX was formed.
  try {
    if (!p2_ || graph_epoch_ < 0)
      return;
    const Key pk = poseKey(graph_epoch_), vk = velKey(graph_epoch_);
    if (!p2_->getLinearizationPoint().exists(pk) ||
        !p2_->getLinearizationPoint().exists(vk))
      return;
    const Mat3 rot =
        p2_->calculateEstimate<gtsam::Pose3>(pk).rotation().matrix();
    const Vec3 v = p2_->calculateEstimate<Vec3>(vk);
    const auto joint = p2_->getISAM2().jointMarginalCovariance({pk, vk});
    const Vec3 lever = stageLever();
    if (!out.position_covariance.allFinite()) {
      Eigen::Matrix<double, 3, 6> J;
      J.leftCols<3>() = -rot * gtsam::skewSymmetric(lever);
      J.rightCols<3>() = rot;
      const gtsam::Matrix6 P = joint.at(pk, pk);
      out.position_covariance = R_ * (J * P * J.transpose()) * R_.transpose();
    }
    // Antenna velocity: body velocity plus the lever-arm rotation term. Its
    // covariance propagates the joint (pose, velocity) marginal through
    // antennaVelocityJacobian, the angular rate taken as known; without the
    // rotation term it is the velocity marginal itself.
    Vec3 lever_rate = Vec3::Zero();
    Mat3 V = joint.at(vk, vk);
    if (!motionStage() && out.angular_velocity.allFinite()) {
      lever_rate = rot * out.angular_velocity.cross(lever);
      Eigen::Matrix<double, 9, 9> P;
      P << joint.at(pk, pk), joint.at(pk, vk), joint.at(vk, pk), V;
      const auto J = antennaVelocityJacobian(rot, out.angular_velocity, lever);
      const Mat3 C = J * P * J.transpose();
      V = .5 * (C + C.transpose());
    }
    out.velocity = R_ * (v + lever_rate);
    out.velocity_covariance = R_ * V * R_.transpose();
  } catch (const std::exception &) {
    // Leave the unavailable quantities NaN.
  }
}
Solution Engine::process(const ObservationEpoch &obs) {
  if (!std::isfinite(obs.gpst) || (last_gpst_ && obs.gpst <= *last_gpst_))
    throw std::invalid_argument("unordered GNSS input");
  if (last_gpst_)
    dt_ = obs.gpst - *last_gpst_;
  last_gpst_ = current_gpst_ = obs.gpst;
  current_ = EpochState{};
  motion_update_ = false;
  integrity_epoch_ = false;
  epoch_doppler_.reset();
  switch_reject_ = -1;
  epoch_hold_fix_.reset();
  held_only_float_.reset();
  // gnss_imu_fgo motion stage: the IMU samples up to this epoch are consumed
  // here, whatever the epoch's outcome; the switch window keeps them.
  struct ObservationScope {
    const ObservationEpoch *&slot;
    std::optional<DopplerRows> &doppler; // points into the observation
    ~ObservationScope() {
      slot = nullptr;
      doppler.reset();
    }
  } scope{epoch_observation_ = &obs, epoch_doppler_};
  epoch_imu_.clear();
  if (!cfg_.gnss_motion && phase_ == 1)
    epoch_imu_ = collectImu(obs.gpst);
  const bool gnss = obs.sats.size() >= 4;
  if (gnss)
    ++gnss_epochs_;
  if (!gnss) {
    std::set<SF> expired = obs.input_slips;
    for (auto &[sf, s] : sats_.track)
      if (++s.outage == 6)
        expired.insert(sf);
    for (auto sf : expired)
      sats_.get(sf).invalidate();
    if (!healthy_ || (motionStage() && !graph_gpst_))
      return finalize({});
  }
  try {
    if (!healthy_)
      throw std::runtime_error("previous graph update failed");
    Solution solution = phase2(obs, gnss);
    if (!motionStage())
      if (const int cause = integrity(gnss))
        solution = fallback(obs, gnss, cause);
    // Float restart after the epoch's last solve (fallback() solves it
    // again): gnss_fgo only, the node without an IMU graph.
    if (cfg_.gnss_motion && gnss && epoch_hold_fix_)
      restartFloatArcs();
    return finalize(solution);
  } catch (const std::exception &ex) {
    Solution out;
    out.failure = ex.what();
    if (!gnss && dynamic_cast<const ImuIntervalError *>(&ex))
      return finalize(out);
    if (gnss) {
      try {
        gtsam::Pose3 initial(gtsam::Rot3(),
                             R_.transpose() * (nav_position_ - base_));
        // The IMU graph restarts with its last committed attitude (the body
        // origin at the antenna minus the lever arm).
        if (!motionStage() && last_rotation_)
          initial = gtsam::Pose3(*last_rotation_,
                                 R_.transpose() * (nav_position_ - base_) -
                                     last_rotation_->matrix() * stageLever());
        auto dd = ddprPosition(obs, initial, stageLever());
        if (dd.position && std::isfinite(dd.rms) &&
            dd.rms <= cfg_.ddpr_max_res) {
          warmReset(*dd.position, initial.rotation());
          // The motion stage consumed this epoch's IMU at its start.
          if (!motionStage())
            collectImu(obs.gpst);
          out.position = nav_position_;
          out.valid = true;
          out.kind = UpdateKind::Gnss;
          return finalize(out);
        }
      } catch (const std::exception &) {
      }
      healthy_ = false;
    } else
      healthy_ = false;
    return finalize(out);
  }
}
std::vector<ImuSample> Engine::collectImu(double until) {
  std::vector<ImuSample> result;
  while (imu_index_ < imu_.size() && imu_[imu_index_].gpst <= until + 1e-6)
    result.push_back(imu_[imu_index_++]);
  return result;
}
Engine::LevelImu Engine::levelImu() const {
  if (saved_level_)
    return *saved_level_;
  LevelImu out;
  // Normal gravity at the current antenna position.
  out.b0 = stationaryBias(imu_, std::min<std::size_t>(3000, imu_index_),
                          gravityAt(nav_position_));
  if (leadingStillSamples(imu_, std::min<std::size_t>(3000, imu_index_)) >
      kStillBiasSamples)
    out.source = LevelImu::Bias::Still;
  const std::size_t still = leadingStillSamples(imu_, imu_index_);
  const std::size_t nstatic =
      std::min<std::size_t>(500, still ? still : imu_index_);
  if (!nstatic)
    return out;
  Vec3 a = Vec3::Zero();
  for (std::size_t i = 0; i < nstatic; ++i)
    a += imu_[i].acc;
  a /= nstatic;
  a -= out.b0.accelerometer();
  double pitch = std::atan2(a.x(), std::hypot(a.y(), a.z())),
         roll = std::atan2(a.y(), a.z());
  if (std::isfinite(cfg_.init_pitch_deg))
    pitch = cfg_.init_pitch_deg * M_PI / 180.;
  out.rotation = attitude(roll, pitch, 0.);
  out.roll = roll;
  out.pitch = pitch;
  out.level = true;
  return out;
}
void Engine::updateSwitchWindow(const Solution &out) {
  // Mean yaw rate of this epoch's IMU interval: the bias-corrected angular
  // rate (b0) about the local vertical of the accelerometer roll/pitch that
  // transition() would use now; body z when no IMU has been consumed yet.
  const LevelImu imu = levelImu();
  double yaw_rate = std::numeric_limits<double>::quiet_NaN();
  if (!epoch_imu_.empty()) {
    const Vec3 up = imu.rotation.transpose() * Vec3::UnitZ();
    Vec3 w = Vec3::Zero();
    for (const auto &sample : epoch_imu_)
      w += sample.gyro;
    w = w / double(epoch_imu_.size()) - imu.b0.gyroscope();
    yaw_rate = up.dot(w) * 180. / M_PI;
  }
  // Horizontal speed of the posterior (output) antenna velocity.
  Vec3 velocity = Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
  double speed = std::numeric_limits<double>::quiet_NaN();
  if (out.valid && out.velocity.allFinite()) {
    velocity = R_.transpose() * out.velocity;
    speed = velocity.head<2>().norm();
  }
  // Why the epoch does not qualify (switch_reject_), 0 when it does:
  // 1 no regular GNSS update of the motion graph (phase2, at least
  // min_dd_for_solve DD rows; not dead reckoning, not a reset epoch),
  // 2 posterior speed below vel_thresh, 3-6 the Doppler-only gate,
  // 7 velocity or position covariance not finite, 8 IMU interval, 9 yaw
  // rate, 10 direction of travel undetermined (on the completing epoch).
  int reject = 0;
  double sigma = .02;
  if (!motion_update_ || !epoch_observation_ || !out.valid ||
      out.kind != UpdateKind::Gnss)
    reject = 1;
  else if (!velocity.allFinite() || !out.velocity_covariance.allFinite())
    reject = 7;
  else if (speed < cfg_.vel_thresh)
    reject = 2;
  else if (doppler_gate_) {
    // Doppler-only gate: the receiver velocity from this epoch's
    // between-satellite Doppler rows alone must move too, in the same
    // direction and at about the same speed as the posterior velocity.
    const auto doppler =
        epoch_doppler_ ? dopplerVelocity(*epoch_doppler_) : std::nullopt;
    if (!doppler)
      reject = 3;
    else {
      const double doppler_speed = doppler->head<2>().norm();
      double dheading = std::atan2(doppler->x(), doppler->y()) -
                        std::atan2(velocity.x(), velocity.y());
      dheading = std::abs(std::remainder(dheading, 2 * M_PI)) * 180. / M_PI;
      if (doppler_speed < cfg_.vel_thresh)
        reject = 4;
      else if (dheading > kSwitchMaxHeadingDiffDeg)
        reject = 5;
      else if (std::abs(doppler_speed - speed) >
               std::max(kSwitchSpeedTolMps, kSwitchSpeedTolFrac * speed))
        reject = 6;
    }
  }
  // Yaw rate below kSwitchMaxYawRateDps (code 9): while the vehicle turns
  // the antenna velocity direction is not the body heading. An epoch without
  // IMU samples fails it.
  if (!reject && !(std::abs(yaw_rate) < kSwitchMaxYawRateDps))
    reject = 9;
  // GPSFactorArm sigma of the output position: 2 cm for a FIX, else the
  // largest per-axis standard deviation of its covariance, at least 5 cm.
  if (!reject && !out.fixed) {
    const Mat3 P = R_.transpose() * out.position_covariance * R_;
    if (P.allFinite() && (P.diagonal().array() >= 0).all())
      sigma = std::max(.05, std::sqrt(P.diagonal().maxCoeff()));
    else
      reject = 7;
  }
  switch_reject_ = reject;
  if (reject) {
    window_.clear();
    return;
  }
  // IMU must cover the interval from the previous window epoch (the checks of
  // integrateImu; the preintegration itself is not used, so any frame does);
  // otherwise this epoch starts a new window.
  if (!window_.empty())
    try {
      integrateImu(cfg_, gtsam::imuBias::ConstantBias(), epoch_imu_,
                   window_.back().time, current_gpst_, ImuFrame{});
    } catch (const std::exception &) {
      window_.clear();
      switch_reject_ = 8;
    }
  window_.push_back({current_gpst_, out.position, sigma, out.fixed, velocity,
                     R_.transpose() * out.velocity_covariance * R_,
                     sats_.keys(), std::move(epoch_imu_),
                     *epoch_observation_});
  // The shortest run of the newest epochs that spans kSwitchWindowTime
  // (eps: GPS times near 1.4e9 s carry 2.4e-7 s rounding).
  constexpr double eps = 1e-3;
  while (window_.size() > 2 &&
         window_.back().time - window_[1].time >= kSwitchWindowTime - eps)
    window_.erase(window_.begin());
  if (window_.size() < 2 ||
      window_.back().time - window_.front().time < kSwitchWindowTime - eps)
    return;
  // Direction of travel. The window spans about 1 s without turning, so the
  // body x axis keeps its direction: the speed change along it from the
  // levelled, bias-compensated specific force has the sign of the GNSS speed
  // change when driving forward and the opposite sign when reversing.
  const auto &first = window_.front(), &last = window_.back();
  const double dv_gnss =
      last.velocity.head<2>().norm() - first.velocity.head<2>().norm();
  double dv_imu = 0, previous = first.time;
  // The levelled frame's up is the local up: normal gravity at the antenna.
  const double gravity = gravityAt(nav_position_);
  // Forward horizontal axis of attitude(roll, pitch, 0): heading 0 = north,
  // the y axis of its ENU frame.
  for (std::size_t i = 1; i < window_.size(); ++i) {
    for (const auto &sample : window_[i].imu) {
      const Vec3 f = imu.rotation * (sample.acc - imu.b0.accelerometer()) -
                     Vec3(0, 0, gravity);
      dv_imu += f.y() * (sample.gpst - previous);
      previous = sample.gpst;
    }
    // Interval end after the last sample (zero-order hold, as integrateImu).
    if (!window_[i].imu.empty()) {
      const Vec3 f =
          imu.rotation * (window_[i].imu.back().acc - imu.b0.accelerometer()) -
          Vec3(0, 0, gravity);
      dv_imu += f.y() * (window_[i].time - previous);
    }
    previous = window_[i].time;
  }
  int direction = 0;
  if (std::abs(dv_gnss) >= kSwitchMinSpeedChange &&
      std::abs(dv_imu) >= kSwitchMinSpeedChange)
    direction = (dv_gnss > 0) == (dv_imu > 0) ? 1 : -1;
  else if (std::all_of(window_.begin(), window_.end(),
                       [](const WindowEpoch &w) {
                         return w.velocity.head<2>().norm() >=
                                kSwitchForwardSpeed;
                       }))
    direction = 2;
  if (!direction) {
    // Undetermined: no switch; the window keeps sliding.
    switch_reject_ = 10;
    window_.erase(window_.begin());
    return;
  }
  try {
    transition(direction == -1);
  } catch (const std::exception &) {
    // The motion stage continues unchanged; the next window is collected
    // from the next epoch on.
    window_.clear();
  }
}
int Engine::integrity(bool gnss) {
  if (!gnss || !integrity_epoch_)
    return 0;
  constexpr double eps = 1e-3; // rounding of GPS times near 1.4e9 s
  // Check 1: the IMU graph's body x axis must point along the Doppler-only
  // velocity. That velocity is the antenna's; the IMU-origin velocity
  // subtracts the lever-arm rotation term with the posterior attitude and the
  // angular rate the Doppler factors of this epoch used.
  bool clean = false;
  try {
    const auto doppler =
        epoch_doppler_ ? dopplerVelocity(*epoch_doppler_) : std::nullopt;
    if (doppler) {
      const Mat3 body = p2_->calculateEstimate<gtsam::Pose3>(
                               poseKey(graph_epoch_))
                            .rotation()
                            .matrix();
      const Vec3 v = *doppler - body * epoch_omega_.cross(lever_);
      if (v.head<2>().norm() >= kSwitchForwardSpeed) {
        const Vec3 x = body.col(0);
        const double diff =
            std::abs(std::remainder(std::atan2(v.x(), v.y()) -
                                        std::atan2(x.x(), x.y()),
                                    2 * M_PI)) *
            180. / M_PI;
        if (diff > kIntegrityMaxHeadingDiffDeg)
          heading_timer_.violate(current_gpst_, kIntegrityTime);
        else {
          heading_timer_.clear();
          clean = true;
        }
      }
    }
  } catch (const std::exception &) {
  }
  // A run whose last counting epoch is more than kIntegrityTime back has
  // lapsed (it restarts at its next violation).
  if (heading_timer_.last &&
      current_gpst_ - *heading_timer_.last > kIntegrityTime)
    heading_timer_.clear();
  if (clean)
    good_bias_ = bias_;
  if (heading_timer_.length() >= kIntegrityTime - eps)
    return 1;
  return 0;
}
Solution Engine::fallback(const ObservationEpoch &obs, bool gnss, int cause) {
  ++n_fallback_;
  fallback_cause_ = cause;
  // The next switch levels with the roll/pitch of this graph's last state
  // (relative to the local level at the antenna); its bias starts from the
  // last estimate of this graph while the check was clean, if any.
  LevelImu level = levelImu();
  if (good_bias_) {
    level.b0 = *good_bias_;
    level.source = LevelImu::Bias::Graph;
  }
  if (last_rotation_) {
    const Vec3 rph = rollPitchHeading(localToBase(nav_position_).transpose() *
                                      last_rotation_->matrix());
    level.roll = rph.x();
    level.pitch = rph.y();
    level.rotation = attitude(level.roll, level.pitch, 0.);
    level.level = true;
  }
  saved_level_ = level;
  // A fresh motion stage, as gnss_fgo starts: new smoother, no ambiguity
  // keys or holds (as a reset), no switch window. The IMU buffer stays.
  p2_ = makeSmoother(cfg_.fls_lag, cfg_, true);
  actual_lag_ = cfg_.fls_lag;
  phase_ = 1;
  graph_epoch_ = -1;
  graph_time_ = 0;
  graph_gpst_.reset();
  holds_.clear();
  for (auto &[sf, s] : sats_.track)
    s.invalidate();
  per_sat_res_.clear();
  pim_break_ = false;
  healthy_ = true;
  window_.clear();
  heading_timer_.clear();
  // This epoch is solved again by the new motion graph (its start path: the
  // DD code position as the seed).
  current_ = EpochState{};
  motion_update_ = integrity_epoch_ = false;
  epoch_doppler_.reset();
  return phase2(obs, gnss);
}
void Engine::transition(bool reverse) {
  const auto saved_sats = sats_;
  const auto saved_epoch = current_;
  const auto saved_created = arc_created_;
  try {
    // Initial bias and roll/pitch (levelImu: from the session's first
    // samples, or what an earlier switch used, with the bias of a fallback).
    const LevelImu level = levelImu();
    if (!level.level)
      throw std::runtime_error("initialization needs IMU");
    auto b0 = level.b0;
    const double pitch = level.pitch, roll = level.roll;
    const int n = window_.size();
    if (n < 2)
      throw std::logic_error("switch window too short");
    std::vector<Vec3> enu;
    for (const auto &w : window_)
      enu.push_back(R_.transpose() * (w.position - base_));
    // Body rotation of every window epoch: roll/pitch local-level at the
    // epoch's antenna, heading = direction of the horizontal posterior
    // antenna velocity in that local frame (plus 180 deg when reversing: the
    // body x axis points against the velocity); the window mean velocity
    // where one is too slow.
    Vec3 mean_velocity = Vec3::Zero();
    for (const auto &w : window_)
      mean_velocity += w.velocity / n;
    double previous_heading = std::atan2(mean_velocity.x(), mean_velocity.y());
    std::vector<Mat3> rots;
    for (int i = 0; i < n; ++i) {
      const auto &w = window_[i];
      const double speed = w.velocity.head<2>().norm();
      const Mat3 tilt = localToBase(w.position);
      const Vec3 local = tilt.transpose() * w.velocity;
      const double heading =
          speed > .01 ? std::atan2(local.x(), local.y()) : previous_heading;
      previous_heading = heading;
      rots.push_back(tilt *
                     attitude(roll, pitch, reverse ? heading + M_PI : heading));
    }
    // The gyro also senses the Earth rotation: a b0 measured as the mean of
    // the leading still samples has the Earth rate in the first window body
    // frame removed. An IMU-graph estimate excludes it already, and the zero
    // b0 of a start without a still block is unknown, not a measurement that
    // contains the Earth rate.
    if (level.source == LevelImu::Bias::Still)
      b0 = gtsam::imuBias::ConstantBias(
          b0.accelerometer(),
          b0.gyroscope() - rots[0].transpose() * earthRate(R_));
    // A b0 measured over the leading still samples gets a tight gyroscope
    // part in the window's bias priors (ConstantBias tangent order:
    // accelerometer, gyroscope).
    const auto bias_noise =
        level.source == LevelImu::Bias::Still
            ? splitNoise(.01, kStillBiasGyroSigma)
            : gtsam::noiseModel::Diagonal::shared_ptr(noise(6, .01));
    // Body states of the window: pose (body origin = antenna minus the lever
    // arm) and the preintegration of every interval (pims[i - 1]: epoch
    // i - 1 to i) in the frame at the body position of epoch i - 1.
    std::vector<gtsam::Pose3> poses;
    for (int i = 0; i < n; ++i)
      poses.emplace_back(gtsam::Rot3(rots[i]), enu[i] - rots[i] * lever_);
    std::vector<IntegratedImu> pims;
    for (int i = 1; i < n; ++i)
      pims.push_back(integrateImu(cfg_, b0, window_[i].imu,
                                  window_[i - 1].time, window_[i].time,
                                  imuFrame(poses[i - 1].translation())));
    const double rstd = cfg_.body_rot_std * M_PI / 180.,
                 min_yaw = 2. * M_PI / 180.;
    Graph graph;
    Values values;
    gtsam::FixedLagSmoother::KeyTimestampMap ts;
    gtsam::NavState previous_state;
    for (int i = 0; i < n; ++i) {
      const auto &w = window_[i];
      const double speed = w.velocity.head<2>().norm();
      const Mat3 &rot = rots[i];
      // IMU-origin velocity: the antenna velocity minus the lever-arm
      // rotation term, with the bias-corrected mean angular rate of the
      // adjacent window interval.
      const Vec3 omega = pims[i == 0 ? 0 : i - 1].mean_gyro - b0.gyroscope();
      const Vec3 velocity = w.velocity - rot * omega.cross(lever_);
      const gtsam::Pose3 &pose = poses[i];
      values.insert(poseKey(i), pose);
      values.insert(velKey(i), velocity);
      values.insert(biasKey(i), b0);
      double yaw = i == 0 ? (speed > .1 ? std::hypot(rstd, .1 / speed) : .5)
                          : std::hypot(rstd, .2 / std::max(speed, .2));
      // Not below the heading sigma of the velocity estimate (its cross-track
      // standard deviation over the speed), nor below 2 degrees.
      if (speed > .01) {
        const Eigen::Vector2d across =
            Eigen::Vector2d(-w.velocity.y(), w.velocity.x()) / speed;
        const double variance =
            across.dot(w.velocity_covariance.topLeftCorner<2, 2>() * across);
        if (std::isfinite(variance) && variance > 0)
          yaw = std::max(yaw, std::sqrt(variance) / speed);
      }
      yaw = std::max(yaw, min_yaw);
      // Translation sigmas: .05/.1 m for a FIX epoch; for a float epoch not
      // below its GPS sigma, so the prior does not claim a better position
      // than the measurement.
      const double floor = w.fixed ? 0. : w.position_sigma;
      if (i == 0)
        graph.addPrior<gtsam::Pose3>(
            poseKey(i), pose,
            poseNoise(.02, .02, yaw, std::max(.05, floor),
                      std::max(.05, floor), std::max(.1, floor)));
      else
        graph.addPrior<gtsam::Pose3>(
            poseKey(i), pose,
            poseNoise(.01, .01, yaw, std::max(.1, floor),
                      std::max(.1, floor), std::max(.1, floor)));
      graph.addPrior<gtsam::Vector3>(velKey(i), velocity, noise(3, .5));
      graph.addPrior<gtsam::imuBias::ConstantBias>(biasKey(i), b0, bias_noise);
      // In the rotating frame the IMU factor's covariance is taken at the
      // nominal predicted attitude (from the window state of epoch i - 1).
      if (i > 0)
        graph.emplace_shared<gtsam::CombinedImuFactor>(
            poseKey(i - 1), velKey(i - 1), poseKey(i), velKey(i),
            biasKey(i - 1), biasKey(i), pims[i - 1].pim,
            pims[i - 1].pim.predict(previous_state, b0).attitude());
      previous_state = gtsam::NavState(pose, velocity);
      graph.emplace_shared<gtsam::GPSFactorArm>(poseKey(i), enu[i], lever_,
                                                noise(3, w.position_sigma));
      ts[poseKey(i)] = ts[velKey(i)] = ts[biasKey(i)] =
          w.time - window_.front().time;
    }
    // Ambiguities: the new smoother gets new keys. A held satellite is
    // released, so its new key is seeded from last_held (sigma .1 cycle);
    // the others restart from code minus carrier (sigma_amb0).
    for (auto &[sf, s] : sats_.track)
      if (s.held)
        s.release();
      else
        s.key.reset();
    // An epoch's carrier phase enters only on the arcs (motion-graph keys)
    // that last to the window end: no key spans a slip or a reset.
    const AmbMap &end_keys = window_.back().keys;
    DdRows rows; // no reader: this graph goes straight into the smoother
    for (int i = 0; i < n; ++i) {
      ObservationEpoch observation = window_[i].observation;
      const AmbMap &keys = window_[i].keys;
      for (auto &sat : observation.sats)
        for (int f = 0; f < 3; ++f) {
          const auto now = keys.find({sat.sat, f});
          const auto end = end_keys.find({sat.sat, f});
          if (now == keys.end() || end == end_keys.end() ||
              now->second != end->second)
            sat.phase[f] = 0;
        }
      buildDd(graph, rows, values, observation, poseKey(i), lever_);
    }
    const double last = window_.back().time - window_.front().time;
    for (auto key : values.keys())
      if (!ts.count(key))
        ts[key] = last;
    double lag = cfg_.fls_lag;
    if (lag + 1e-6 < last + dt_)
      lag = last + 2 * dt_;
    auto smoother = makeSmoother(lag, cfg_, false);
    updateProbe("transition", false);
    smoother->update(graph, values, ts);
    updateProbe("transition", true);
    auto est = smoother->calculateEstimate();
    auto p = est.at<gtsam::Pose3>(poseKey(n - 1));
    auto v = est.at<gtsam::Vector3>(velKey(n - 1));
    auto b = est.at<gtsam::imuBias::ConstantBias>(biasKey(n - 1));
    if (!p.matrix().allFinite() || !v.allFinite() || !b.vector().allFinite())
      throw std::runtime_error("invalid initialization state");
    for (auto sf : current_.consumed)
      sats_.get(sf).seed_pending = false;
    // Commit: nothing below throws.
    nav_position_ = antenna(p, lever_);
    p2_ = std::move(smoother);
    holds_.clear();
    graph_time_ = last;
    graph_epoch_ = n - 1;
    graph_gpst_ = window_.back().time;
    bias_ = initial_bias_ = b0;
    actual_lag_ = lag;
    phase_ = 2;
    healthy_ = true;
    pim_break_ = false;
    window_.clear();
    saved_level_ = level;
    last_rotation_ = p.rotation();
    heading_timer_.clear();
  } catch (...) {
    sats_ = saved_sats;
    current_ = saved_epoch;
    arc_created_ = saved_created;
    throw;
  }
}
void Engine::addImuChain(Graph &graph, Values &values, int k,
                         const IntegratedImu &im, const gtsam::Pose3 &previous,
                         const Vec3 &velocity) {
  if (pim_break_) {
    const auto pred =
        im.pim.predict(gtsam::NavState(previous, velocity), bias_);
    values.update(poseKey(k), pred.pose());
    values.update(velKey(k), gtsam::Vector3(pred.velocity()));
    double t = cfg_.pim_break_trans_sigma;
    graph.addPrior<gtsam::Pose3>(poseKey(k), pred.pose(),
                                 poseNoise(.1, .1, .3, t, t, t));
    graph.addPrior<gtsam::Vector3>(velKey(k), pred.velocity(), noise(3, 2.));
    graph.addPrior<gtsam::imuBias::ConstantBias>(biasKey(k), bias_,
                                                 noise(6, .01));
    pim_break_ = false;
    return;
  }
  // The bias evolves only through the CombinedImuFactor's random walk. In the
  // rotating frame its covariance is taken at the nominal predicted attitude
  // (bias_ as the nominal bias).
  graph.emplace_shared<gtsam::CombinedImuFactor>(
      poseKey(k - 1), velKey(k - 1), poseKey(k), velKey(k), biasKey(k - 1),
      biasKey(k), im.pim,
      im.pim.predict(gtsam::NavState(previous, velocity), bias_).attitude());
}
gtsam::FactorIndices Engine::flsUpdate(const Graph &graph,
                                       const Values &values, int k,
                                       const std::vector<Key> &keep) {
  if (!graph_gpst_ && !motionStage())
    throw std::runtime_error("committed graph time missing");
  const double next =
      graph_gpst_ ? graph_time_ + current_gpst_ - *graph_gpst_ : 0.;
  gtsam::FixedLagSmoother::KeyTimestampMap ts;
  gtsam::FactorIndices slots;
  ts[poseKey(k)] = ts[velKey(k)] = next;
  if (!motionStage())
    ts[biasKey(k)] = next;
  if (k > 0) {
    ts[poseKey(k - 1)] = ts[velKey(k - 1)] = next;
    if (!motionStage())
      ts[biasKey(k - 1)] = next;
  }
  for (auto key : values.keys())
    ts[key] = next;
  for (auto key : keep)
    ts[key] = next;
  try {
    updateProbe("fls", false);
    p2_->update(graph, values, ts);
    updateProbe("fls", true);
    slots = p2_->getISAM2Result().newFactorsIndices;
  } catch (...) {
    healthy_ = false;
    throw;
  }
  for (auto sf : current_.consumed)
    sats_.get(sf).seed_pending = false;
  graph_time_ = next;
  graph_epoch_ = k;
  graph_gpst_ = current_gpst_;
  return slots;
}
void Engine::addPseudo(Graph &graph, Values &values, int k,
                       const Vec3 &velocity,
                       const std::vector<ImuSample> &samples, const Vec3 &gyro,
                       bool bore_live) {
  const double speed = velocity.head<2>().norm();
  if (cfg_.nhc_enable && speed >= cfg_.nhc_min_speed) {
    Vec3 lever = Vec3::Zero();
    std::string text = cfg_.nhc_lever;
    std::replace(text.begin(), text.end(), ',', ' ');
    std::istringstream stream(text);
    stream >> lever.x() >> lever.y() >> lever.z();
    // The mounting pitch enters the IMU graph with its first NHC factor, at
    // its last estimate (zero before any), with the prior N(0, sigma).
    if (!bore_live) {
      values.insert(boreKey(), last_bore_.value_or(0.));
      graph.addPrior<double>(boreKey(), 0.,
                             noise(1, kMountingPitchSigmaDeg * M_PI / 180.));
    }
    graph.emplace_shared<NhcBoreFactor>(
        poseKey(k), velKey(k), boreKey(), gyro - bias_.gyroscope(), lever,
        speed, cfg_.nhc_sigma_lat, cfg_.nhc_sigma_vert);
  }
  bool stationary =
      cfg_.zupt_enable && int(samples.size()) >= cfg_.zupt_min_samples;
  if (stationary) {
    Vec3 acc = Vec3::Zero(), gyro_mean = Vec3::Zero();
    for (auto &s : samples) {
      acc += s.acc;
      gyro_mean += s.gyro;
    }
    acc /= samples.size();
    gyro_mean /= samples.size();
    double av = 0, gv = 0;
    std::vector<double> gn;
    for (auto &s : samples) {
      av += (s.acc - acc).squaredNorm();
      gv += (s.gyro - gyro_mean).squaredNorm();
      gn.push_back((s.gyro - initial_bias_.gyroscope()).norm());
    }
    stationary = std::sqrt(av / samples.size()) <= cfg_.zupt_max_acc_std &&
                 std::sqrt(gv / samples.size()) <= cfg_.zupt_max_gyro_std &&
                 median(gn) <= cfg_.zupt_max_gyro_median;
  }
  // Speed gate: the epoch's Doppler-only speed (the rows addDoppler used
  // earlier in this phase2 call); the previous posterior speed when no
  // Doppler-only velocity is formed.
  std::optional<double> doppler_speed;
  if (epoch_doppler_)
    if (const auto v = dopplerVelocity(*epoch_doppler_))
      doppler_speed = v->norm();
  if (doppler_speed)
    stationary = stationary && *doppler_speed < kZuptDopplerSpeed;
  else
    stationary = stationary && velocity.norm() < kZuptMaxSpeed;
  if (!stationary)
    return;
  // Zero velocity, and zero relative motion since the previous epoch (Pose3
  // tangent order: rotation, translation). No absolute pose anchor: the
  // GNSS factors keep the position.
  graph.addPrior<gtsam::Vector3>(velKey(k), Vec3::Zero(),
                                 noise(3, cfg_.zupt_sigma_zero_velocity));
  if (k > 0)
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        poseKey(k - 1), poseKey(k), gtsam::Pose3(),
        splitNoise(kZuptRotationSigma, kZuptTranslationSigma));
}
std::optional<Engine::DopplerRows>
Engine::dopplerRows(const ObservationEpoch &obs, const gtsam::NavState &pred) {
  std::vector<DopplerRow> rows;
  for (const auto &s : obs.sats) {
    if (s.system == 4)
      continue;
    for (int f = 0; f < 3; ++f)
      if (std::isfinite(s.doppler[f]) && s.doppler_wavelength[f] > 0) {
        rows.push_back(
            {&s, s.doppler[f], s.doppler_wavelength[f], s.doppler_snr[f]});
        break;
      }
  }
  if (rows.size() < 2)
    return std::nullopt;
  const Vec3 receiver = antenna(pred.pose()), vpred = R_ * pred.velocity();
  double scale = 0;
  if (rows.size() >= 5) {
    Eigen::MatrixXd A(rows.size(), 4);
    Eigen::VectorXd b(rows.size());
    for (std::size_t i = 0; i < rows.size(); ++i) {
      const auto &row = rows[i];
      Vec3 e = (row.sat->rover_sat - receiver).normalized();
      A.block<1, 3>(i, 0) = -e.transpose();
      A(i, 3) = 1.;
      b[i] = -row.lam * row.d - e.dot(row.sat->sat_velocity - vpred) +
             299792458. * row.sat->sat_clock_drift;
    }
    Eigen::VectorXd x =
        A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    Eigen::VectorXd residual = A * x - b;
    std::vector<double> r(residual.data(), residual.data() + residual.size());
    const double mid = median(r);
    for (auto &val : r)
      val = std::abs(val - mid);
    scale = 1.4826 * median(r);
    if (scale > 0) {
      std::vector<DopplerRow> kept;
      for (std::size_t i = 0; i < rows.size(); ++i)
        if (std::abs(residual[i]) <= cfg_.doppler_fde_k * scale)
          kept.push_back(rows[i]);
      if (kept.size() >= 4)
        rows = std::move(kept);
    }
  }
  DopplerRows out;
  for (auto &row : rows) {
    double sigma = std::max(cfg_.doppler_sd_sigma, scale);
    sigma /= std::max(std::sin(row.sat->elevation), .1);
    if (row.snr > 0)
      sigma *= std::clamp(std::pow(10., (cfg_.snr_ref_dbhz - row.snr) / 20.),
                          1., 10.);
    out.sigmas.push_back(sigma);
  }
  out.ref = std::min_element(out.sigmas.begin(), out.sigmas.end()) -
            out.sigmas.begin();
  out.rows = std::move(rows);
  out.receiver = receiver;
  return out;
}
std::optional<Vec3> Engine::dopplerVelocity(const DopplerRows &d) const {
  const std::size_t n = d.rows.size();
  if (n < 5) // fewer than 4 between-satellite rows
    return std::nullopt;
  // Each row is the SdDopplerFactor of addDoppler with no lever arm:
  // error = constant + h . (R v), linear in the ENU velocity v; whitened by
  // the factor's sigma (no Huber: this is a gate, not an estimate).
  Eigen::MatrixXd A(n - 1, 3);
  Eigen::VectorXd b(n - 1);
  const auto &ref = d.rows[d.ref];
  for (std::size_t i = 0, r = 0; i < n; ++i) {
    if (i == d.ref)
      continue;
    const auto &row = d.rows[i];
    const SdDopplerFactor f(
        poseKey(0), velKey(0), row.d, ref.d, row.lam, ref.lam,
        row.sat->rover_sat, row.sat->sat_velocity, ref.sat->rover_sat,
        ref.sat->sat_velocity, d.receiver, Vec3::Zero(), R_, Vec3::Zero(),
        noise(1, 1.), row.sat->sat_clock_drift, ref.sat->sat_clock_drift);
    const double w = 1. / std::hypot(d.sigmas[i], d.sigmas[d.ref]);
    A.row(r) = w * f.h.transpose() * R_;
    b[r] = -w * f.constant;
    ++r;
  }
  const auto qr = A.colPivHouseholderQr();
  if (qr.rank() < 3)
    return std::nullopt;
  const Vec3 v = qr.solve(b);
  if (!v.allFinite())
    return std::nullopt;
  return v;
}
void Engine::addDoppler(Graph &graph, int k, const ObservationEpoch &obs,
                        const gtsam::NavState &pred, const Vec3 &gyro, int nv) {
  if (cfg_.doppler_sd_sigma <= 0 || nv < cfg_.min_dd_for_solve)
    return;
  auto doppler = dopplerRows(obs, pred);
  if (!doppler)
    return;
  const auto &rows = doppler->rows;
  const auto &sigmas = doppler->sigmas;
  const std::size_t ref_index = doppler->ref;
  const Vec3 &receiver = doppler->receiver;
  const auto &ref = rows[ref_index];
  for (std::size_t i = 0; i < rows.size(); ++i) {
    if (i == ref_index)
      continue;
    const auto &row = rows[i];
    gtsam::SharedNoiseModel model =
        noise(1, std::hypot(sigmas[i], sigmas[ref_index]));
    model = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(cfg_.doppler_huber),
        model);
    graph.emplace_shared<SdDopplerFactor>(
        poseKey(k), velKey(k), row.d, ref.d, row.lam, ref.lam,
        row.sat->rover_sat, row.sat->sat_velocity, ref.sat->rover_sat,
        ref.sat->sat_velocity, receiver, stageLever(), R_,
        gyro - bias_.gyroscope(),
        model, row.sat->sat_clock_drift, ref.sat->sat_clock_drift);
  }
  // gnss_imu_fgo: the same rows gate the switch window (motion stage) and
  // feed integrity check 1 (IMU stage, with the factors' angular rate).
  if (!cfg_.gnss_motion) {
    epoch_doppler_ = std::move(doppler);
    epoch_omega_ = gyro - bias_.gyroscope();
  }
}
std::map<int, double> Engine::residuals(const Graph &graph, const DdRows &rows,
                                       const Values &values) const {
  std::map<int, double> per_sat;
  for (const auto &row : rows) {
    if (row.carrier)
      continue;
    const double r = std::sqrt(2. * std::max(0., gaussianError(
                                                  *graph.at(row.index), values))) *
                     cfg_.sigma_pr * std::sqrt(2.);
    for (int sat : {row.ref_sat, row.target_sat})
      if (r > per_sat[sat])
        per_sat[sat] = r;
  }
  return per_sat;
}
void Engine::warmReset(const Vec3 &position, const gtsam::Rot3 &rotation) {
  if (motionStage()) {
    auto replacement = makeSmoother(actual_lag_, cfg_, true);
    Graph graph;
    Values values;
    gtsam::Pose3 pose(gtsam::Rot3(), R_.transpose() * (position - base_));
    values.insert(poseKey(0), pose);
    values.insert(velKey(0), Vec3::Zero().eval());
    graph.addPrior<gtsam::Pose3>(poseKey(0), pose,
                                 poseNoise(.01, .01, .01, 100, 100, 100));
    graph.addPrior<Vec3>(velKey(0), Vec3::Zero(), noise(3, 100));
    updateProbe("warm_reset", false);
    replacement->update(graph, values, {{poseKey(0), 0.}, {velKey(0), 0.}});
    updateProbe("warm_reset", true);
    const auto estimate = replacement->calculateEstimate();
    if (!estimate.at<gtsam::Pose3>(poseKey(0)).matrix().allFinite() ||
        !estimate.at<Vec3>(velKey(0)).allFinite())
      throw std::runtime_error("invalid motion reset");
    p2_ = std::move(replacement);
    holds_.clear();
    graph_epoch_ = 0;
    graph_time_ = 0.;
    graph_gpst_ = current_gpst_;
    healthy_ = true;
    for (auto &[sf, s] : sats_.track)
      s.invalidate();
    current_.accepted.clear();
    current_.visible.clear();
    per_sat_res_.clear();
    pim_break_ = false;
    nav_position_ = position;
    return;
  }
  gtsam::Pose3 pose(rotation, R_.transpose() * (position - base_));
  auto replacement = makeSmoother(actual_lag_, cfg_, false);
  Graph graph;
  Values values;
  values.insert(poseKey(0), pose);
  values.insert(velKey(0), Vec3::Zero().eval());
  values.insert(biasKey(0), bias_);
  double rot = cfg_.body_rot_std * M_PI / 180.;
  graph.addPrior<gtsam::Pose3>(poseKey(0), pose,
                               poseNoise(rot, rot, .3, 2, 2, 3));
  graph.addPrior<gtsam::Vector3>(velKey(0), Vec3::Zero(), noise(3, 3));
  graph.addPrior<gtsam::imuBias::ConstantBias>(biasKey(0), bias_,
                                               noise(6, .01));
  gtsam::FixedLagSmoother::KeyTimestampMap ts;
  for (auto k : values.keys())
    ts[k] = dt_;
  updateProbe("warm_reset", false);
  replacement->update(graph, values, ts);
  updateProbe("warm_reset", true);
  auto estimate = replacement->calculateEstimate();
  if (!estimate.at<gtsam::Pose3>(poseKey(0)).matrix().allFinite() ||
      !estimate.at<Vec3>(velKey(0)).allFinite() ||
      !estimate.at<gtsam::imuBias::ConstantBias>(biasKey(0))
           .vector()
           .allFinite())
    throw std::runtime_error("invalid reset state");
  p2_ = std::move(replacement);
  holds_.clear();
  graph_time_ = dt_;
  graph_epoch_ = 0;
  graph_gpst_ = current_gpst_;
  healthy_ = true;
  for (auto &[sf, s] : sats_.track)
    s.invalidate();
  per_sat_res_.clear();
  current_.accepted.clear();
  current_.visible.clear();
  pim_break_ = true;
  nav_position_ = antenna(pose);
  last_rotation_ = pose.rotation();
}
// One key per ambiguity arc: a key carries over from epoch to epoch with no
// new prior; the carrier DDs of all its epochs share it.
// An arc ends (the next carrier DD starts a new key) on:
//  - a slip or outage: LLI, input slip, geometry-free jump, or 6 epochs in a
//    row without the band's carrier on both receivers (slips(); process()
//    for epochs without GNSS);
//  - a key the smoother no longer has (continueArcs());
//  - a GNSS epoch without a carrier DD of the satellite/band (endUnusedArcs());
//  - a hold release without a seed: after a GNSS epoch without a fix, the
//    held band with the largest residual above a threshold (phase2()); the
//    holds a rejected carrier DD contradicts (fde());
//  - gnss_fgo's float restart of the un-held arcs (restartFloatArcs());
//  - a reset (warmReset(), fallback()) or the switch to the IMU graph
//    (transition()), which end every arc.
// A hold the carrier data contradict is dropped without ending its arc
// (rejectHolds()).
void Engine::continueArcs(const Values &estimate) {
  for (auto &[sf, s] : sats_.track) {
    if (s.key && !estimate.exists(*s.key))
      s.endArc();
    // A hold lives on its arc's key. A held ambiguity whose key does not
    // reach this epoch is released, so its next key is seeded from last_held.
    if (s.held && !s.key)
      s.release();
  }
}
// After buildDd: a key without a carrier DD this epoch ends its arc. A held
// one stays held until the next GNSS epoch releases it (continueArcs).
void Engine::endUnusedArcs() {
  for (auto &[sf, s] : sats_.track)
    if (s.key && !current_.visible.count(sf))
      s.endArc();
}
Solution Engine::phase2(const ObservationEpoch &obs, bool gnss) {
  const int k = graph_epoch_ + 1;
  epoch_hold_fix_.reset();
  held_only_float_.reset();
  Values estimate;
  gtsam::Pose3 previous;
  Vec3 velocity = Vec3::Zero();
  gtsam::imuBias::ConstantBias previous_bias;
  gtsam::NavState pred;
  Graph graph;
  Values values;
  std::vector<ImuSample> samples;
  // The motion stage reads only its (zero) mean angular rate; the IMU stage
  // replaces it with the epoch's preintegration.
  IntegratedImu integrated(cfg_, bias_, ImuFrame{});
  // For the plausibility guard (kMaxSpeed): the last committed antenna
  // position and its time; a new motion graph is checked against its DD code
  // seed.
  Vec3 committed = nav_position_;
  const double committed_gpst = graph_gpst_.value_or(obs.gpst);
  if (motionStage()) {
    if (!graph_gpst_) {
      gtsam::Pose3 seed(gtsam::Rot3(),
                        R_.transpose() * (nav_position_ - base_));
      const auto dd = ddprPosition(obs, seed, Vec3::Zero());
      if (!dd.position || !dd.position->allFinite())
        return {};
      committed = *dd.position;
      seed = gtsam::Pose3(gtsam::Rot3(),
                          R_.transpose() * (*dd.position - base_));
      pred = gtsam::NavState(seed, Vec3::Zero());
      graph.addPrior<gtsam::Pose3>(poseKey(k), seed,
                                   poseNoise(.01, .01, .01, 100, 100, 100));
      graph.addPrior<Vec3>(velKey(k), Vec3::Zero(), noise(3, 100));
    } else {
      estimate = p2_->calculateEstimate();
      previous = estimate.at<gtsam::Pose3>(poseKey(k - 1));
      velocity = estimate.at<Vec3>(velKey(k - 1));
      const double elapsed = obs.gpst - *graph_gpst_;
      pred = gtsam::NavState(
          gtsam::Pose3(gtsam::Rot3(),
                       previous.translation() + elapsed * velocity),
          velocity);
      graph.emplace_shared<MotionFactor>(poseKey(k - 1), velKey(k - 1),
                                         poseKey(k), velKey(k), elapsed,
                                         cfg_.motion_acceleration_psd);
      graph.emplace_shared<gtsam::PoseRotationPrior<gtsam::Pose3>>(
          poseKey(k), gtsam::Rot3(), noise(3, .01));
    }
    values.insert(poseKey(k), pred.pose());
    values.insert(velKey(k), Vec3(pred.velocity()));
  } else {
    estimate = p2_->calculateEstimate();
    // The interval's frame is the one at the previous body position.
    const Vec3 from = estimate.at<gtsam::Pose3>(poseKey(k - 1)).translation();
    const auto cursor = imu_index_;
    samples = collectImu(obs.gpst);
    try {
      integrated = integrateImu(
          cfg_, bias_, samples,
          graph_gpst_.value_or(std::numeric_limits<double>::quiet_NaN()),
          obs.gpst, imuFrame(from));
    } catch (...) {
      imu_index_ = cursor;
      throw;
    }
    previous = estimate.at<gtsam::Pose3>(poseKey(k - 1));
    velocity = estimate.at<Vec3>(velKey(k - 1));
    previous_bias =
        estimate.at<gtsam::imuBias::ConstantBias>(biasKey(k - 1));
    pred = integrated.pim.predict(gtsam::NavState(previous, velocity),
                                  previous_bias);
    values.insert(poseKey(k), pred.pose());
    values.insert(velKey(k), Vec3(pred.velocity()));
    values.insert(biasKey(k), previous_bias);
    addImuChain(graph, values, k, integrated, previous, velocity);
  }
  int nv = 0;
  DdRows rows;
  if (gnss) {
    slips(obs);
    continueArcs(estimate);
    nv = buildDd(graph, rows, values, obs, poseKey(k), stageLever());
    endUnusedArcs();
    if (nv < cfg_.min_dd_for_solve && !motionStage()) {
      graph.addPrior<gtsam::Pose3>(poseKey(k), pred.pose(),
                                   noise(6, cfg_.propagate_pose_sigma));
      graph.addPrior<Vec3>(velKey(k), pred.velocity(),
                           noise(3, cfg_.propagate_vel_sigma));
      graph.addPrior<gtsam::imuBias::ConstantBias>(
          biasKey(k), previous_bias, noise(6, cfg_.propagate_bias_sigma));
    }
    addDoppler(graph, k, obs, pred, integrated.mean_gyro, nv);
  } else if (!motionStage())
    graph.addPrior<gtsam::imuBias::ConstantBias>(biasKey(k), bias_,
                                                 splitNoise(1e-4, 3e-6));
  // The mounting-pitch key is live when the pre-update estimate of this IMU
  // graph has it; a live key is refreshed every update, whether or not this
  // epoch adds an NHC factor.
  const bool bore_live = !motionStage() && estimate.exists(boreKey());
  if (!motionStage())
    addPseudo(graph, values, k, velocity, samples, integrated.mean_gyro,
              bore_live);
  // Every key of a live arc is refreshed, also in epochs without GNSS; an
  // ended arc's key is not and leaves through the lag.
  std::vector<Key> keep;
  for (auto [sf, key] : sats_.keys())
    keep.push_back(key);
  if (bore_live)
    keep.push_back(boreKey());
  gtsam::FactorIndices slots;
  if (graph_observer_) {
    Values all(estimate);
    all.insert(values);
    graph_observer_(input_epoch_, graph, all);
  }
  try {
    slots = flsUpdate(graph, values, k, keep);
    estimate = p2_->calculateEstimate();
  } catch (const std::exception &ex) {
    healthy_ = false;
    if (!gnss)
      throw;
    Solution out;
    out.failure = ex.what();
    // Seeded at the last committed position: `pred` comes from the graph
    // that failed and may be implausible itself.
    const gtsam::Rot3 rotation = pred.pose().rotation();
    const auto dd = ddprPosition(
        obs,
        gtsam::Pose3(rotation, R_.transpose() * (committed - base_) -
                                   rotation.matrix() * stageLever()),
        stageLever());
    if (dd.position && std::isfinite(dd.rms) && dd.rms <= cfg_.ddpr_max_res) {
      try {
        warmReset(*dd.position, rotation);
      } catch (const std::exception &reset_error) {
        healthy_ = false;
        out.failure += std::string("; reset: ") + reset_error.what();
        return out;
      }
      out.position = nav_position_;
      out.valid = true;
      out.kind = UpdateKind::Gnss;
    }
    return out;
  }
  // Satellites with a code DD residual, in the order of the rows.
  std::vector<int> residual_sat_order;
  if (gnss) {
    ar_per_sat_res_ = residuals(graph, rows, estimate);
    std::set<int> seen;
    for (const auto &row : rows) {
      if (row.carrier)
        continue;
      for (int s : {row.ref_sat, row.target_sat})
        if (ar_per_sat_res_.count(s) && seen.insert(s).second)
          residual_sat_order.push_back(s);
    }
    estimate = fde(*p2_, graph, rows, slots, estimate, poseKey(k));
    estimate = rejectHolds(*p2_, std::move(estimate), poseKey(k));
  }
  auto pose = estimate.at<gtsam::Pose3>(poseKey(k));
  auto vel = estimate.at<Vec3>(velKey(k));
  const auto bias = motionStage()
                        ? bias_
                        : estimate.at<gtsam::imuBias::ConstantBias>(biasKey(k));
  if (!pose.matrix().allFinite() || !vel.allFinite() ||
      !bias.vector().allFinite())
    throw std::runtime_error("nonfinite TC posterior");
  // An implausible posterior (an indeterminate graph returns one without an
  // error) is a failed update as well: process() restarts from the DD code
  // position, and nav_position_, from which the node computes the elevation
  // mask, keeps the last committed position.
  if (plausibility_guard_ &&
      ((antenna(pose) - committed).norm() >
           kMaxSpeed * (obs.gpst - committed_gpst) + kMaxStepMargin ||
       vel.norm() > kMaxSpeed))
    throw std::runtime_error("implausible TC posterior");
  bias_ = bias;
  nav_position_ = antenna(pose);
  if (!motionStage()) {
    last_rotation_ = pose.rotation();
    if (estimate.exists(boreKey()))
      last_bore_ = estimate.at<double>(boreKey());
  }
  if (gnss)
    epoch_held_arcs_ = heldArcs(searchArcs(obs, estimate));
  ArDecision ar;
  if (gnss && k >= (motionStage() ? 5 : cfg_.n_collect + 3))
    ar = runAr(*p2_, estimate, poseKey(k), obs);
  // A held-only fix sets the output alone: for the holds and arcs the epoch
  // is a float epoch.
  const bool hold_fix = ar.nb > 0 && !ar.held_only;
  if (hold_fix)
    hold(*p2_, ar);
  else if (gnss) {
    const double threshold = std::max(2., .5 * cfg_.ar_context_worst_sat_max);
    int worst = 0;
    double worst_res = 0;
    for (auto s : residual_sat_order)
      if (residualOf(ar_per_sat_res_, s) > worst_res) {
        worst = s;
        worst_res = residualOf(ar_per_sat_res_, s);
      }
    std::optional<std::tuple<double, double, int, int>> best;
    for (auto sf : sats_.insertion_order)
      if (sats_.get(sf).held) {
        double r = residualOf(ar_per_sat_res_, sf.first),
               score = r >= threshold ? r : 0.;
        if (sf.first == worst && worst_res >= threshold)
          score += std::max(1., .25 * worst_res);
        auto candidate = std::make_tuple(score, r, sf.first, sf.second);
        if (score > 0 && (!best || candidate > *best))
          best = candidate;
      }
    if (best)
      restartArc(sats_.get({std::get<2>(*best), std::get<3>(*best)}));
  }
  Solution sol;
  sol.position = ar.nb ? ar.fixed_position : nav_position_;
  if (ar.nb)
    sol.position_covariance = R_ * ar.fixed_covariance * R_.transpose();
  sol.valid = true;
  sol.fixed = ar.nb > 0;
  sol.nb = ar.nb;
  sol.ratio = ar.s0 > 0 ? ar.s1 / ar.s0 : 0;
  sol.kind = gnss && nv ? UpdateKind::Gnss : UpdateKind::DeadReckoning;
  if (gnss)
    per_sat_res_ = ar_per_sat_res_;
  motion_update_ =
      motionStage() && gnss && nv >= cfg_.min_dd_for_solve;
  integrity_epoch_ = !motionStage();
  if (ar.held_only && ar.nb)
    held_only_float_ = nav_position_;
  if (gnss)
    epoch_hold_fix_ = hold_fix;
  return sol;
}
} // namespace gnss_fgo
