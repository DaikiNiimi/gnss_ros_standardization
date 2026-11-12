#include "engine.hpp"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <gtsam/linear/HessianFactor.h>
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
} // namespace
gtsam::NonlinearFactor::shared_ptr
marginalWithout(const gtsam::LinearContainerFactor &f,
                const std::set<Key> &drop) {
  const gtsam::HessianFactor h(*f.factor());
  std::vector<Eigen::Index> kept, dropped;
  gtsam::KeyVector keys;
  std::vector<std::size_t> dims;
  Eigen::Index offset = 0;
  for (auto it = h.begin(); it != h.end(); ++it) {
    const Eigen::Index d = h.getDim(it);
    auto &cols = drop.count(*it) ? dropped : kept;
    for (Eigen::Index i = 0; i < d; ++i)
      cols.push_back(offset + i);
    if (!drop.count(*it)) {
      keys.push_back(*it);
      dims.push_back(d);
    }
    offset += d;
  }
  if (dropped.empty() || kept.empty())
    return nullptr;
  const gtsam::Matrix G = h.information();
  const gtsam::Vector g = h.linearTerm();
  const gtsam::Matrix Gkk = G(kept, kept), Gka = G(kept, dropped),
                     Gaa = G(dropped, dropped);
  // Pseudo-inverse of the dropped block: eigenvalues at or below 1e-12 times
  // the largest are directions `f` does not determine and are skipped.
  const Eigen::SelfAdjointEigenSolver<gtsam::Matrix> eig(Gaa);
  const gtsam::Vector &l = eig.eigenvalues();
  gtsam::Vector inverse = gtsam::Vector::Zero(l.size());
  for (Eigen::Index i = 0; i < l.size(); ++i)
    if (l[i] > 1e-12 * l.cwiseAbs().maxCoeff())
      inverse[i] = 1. / l[i];
  const gtsam::Matrix pinv = eig.eigenvectors() * inverse.asDiagonal() *
                             eig.eigenvectors().transpose();
  // Augmented information [G' g'; g'^T c'] of the Schur complement.
  const Eigen::Index n = kept.size();
  gtsam::Matrix augmented(n + 1, n + 1);
  augmented.topLeftCorner(n, n) = Gkk - Gka * pinv * Gka.transpose();
  augmented.topRightCorner(n, 1) =
      g(kept) - Gka * pinv * gtsam::Vector(g(dropped));
  augmented(n, n) =
      h.constantTerm() -
      gtsam::Vector(g(dropped)).dot(pinv * gtsam::Vector(g(dropped)));
  augmented.bottomLeftCorner(1, n) = augmented.topRightCorner(n, 1).transpose();
  dims.push_back(1);
  const gtsam::HessianFactor marginal(
      keys, gtsam::SymmetricBlockMatrix(dims, augmented));
  gtsam::Values point;
  if (f.linearizationPoint())
    for (auto key : keys)
      point.insert(key, f.linearizationPoint()->at(key));
  return std::make_shared<gtsam::LinearContainerFactor>(marginal, point);
}
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
int Engine::buildDd(Graph &graph, DdRows &rows, Values &values,
                    const ObservationEpoch &obs, Key pose, const Vec3 &lever,
                    bool skip_cp) {
  current_.visible.clear();
  int nv = 0;
  // For the gauge prior below: the keys created in this call.
  std::set<Key> created;
  for (const auto &[sys, idx] : groups(obs)) {
    if (idx.size() < 2)
      continue;
    const int ri = reference(obs, idx);
    const auto &r = obs.sats[ri];
    current_.references[sys] = r.sat;
    // Per band: carrier DDs added, one of their keys existed before this
    // call, one of their keys was seeded from a held value.
    std::array<bool, 3> carrier{}, continuing{}, seeded{};
    for (int ji : idx) {
      if (ji == ri)
        continue;
      const auto &j = obs.sats[ji];
      if (cfg_.exclude_bds_geo && geo(j))
        continue;
      for (int f = 0; f < 3; ++f) {
        if (!r.pr[f] || !r.pr_base[f] || !j.pr[f] || !j.pr_base[f])
          continue;
        // Raw pseudoranges [m]; error = predicted DD range - measured DD.
        const double pr_rov_ref = r.pr[f], pr_base_ref = r.pr_base[f],
                     pr_rov_target = j.pr[f], pr_base_target = j.pr_base[f];
        rows.push_back({graph.size(), false, r.sat, j.sat, f});
        graph.emplace_shared<gtsam::DoubleDifferencePseudorangeFactorArm>(
            pose, pr_rov_ref, pr_base_ref, pr_rov_target, pr_base_target,
            r.rover_sat, j.rover_sat, r.base_sat, j.base_sat, base_, lever,
            frame_,
            gtsam::noiseModel::Isotropic::Sigma(
                1, ddSigma(cfg_, true, r.elevation, j.elevation,
                           obs.gpst - obs.base_gpst)));
        ++nv;
        const double lam = r.wavelength[f];
        if (skip_cp || sys == 4 || lam <= 0 || !r.phase[f] ||
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
        // Raw carrier phases in metres (cycles * lam); ambiguities in cycles;
        // error = predicted DD range + lam (N_ref - N_target) - DD. A held
        // ambiguity keeps its key; the hold is a separate factor (hold()).
        const double cp_rov_ref = r.phase[f] * lam,
                     cp_base_ref = r.phase_base[f] * lam,
                     cp_rov_target = j.phase[f] * lam,
                     cp_base_target = j.phase_base[f] * lam;
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
  for (const auto &[sys, idx] : groups(obs)) {
    if (idx.size() < 2)
      continue;
    int ri = reference(obs, idx);
    const auto &r = obs.sats[ri];
    for (int ji : idx) {
      if (ji == ri)
        continue;
      const auto &j = obs.sats[ji];
      for (int f = 0; f < 3; ++f) {
        if (!r.pr[f] || !r.pr_base[f] || !j.pr[f] || !j.pr_base[f])
          continue;
        factors.push_back(
            std::make_shared<gtsam::DoubleDifferencePseudorangeFactorArm>(
                key, r.pr[f], r.pr_base[f], j.pr[f], j.pr_base[f], r.rover_sat,
                j.rover_sat, r.base_sat, j.base_sat, base_, lever, frame_,
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
  gtsam::FactorIndices rejected;
  for (const auto &row : rows) {
    double res = std::sqrt(2 * graph[row.index]->error(est)) *
                 (row.carrier ? cfg_.sigma_cp : cfg_.sigma_pr) * std::sqrt(2.);
    if (res > (row.carrier ? cfg_.fde_cp : cfg_.fde_pr))
      rejected.push_back(slots[row.index]);
  }
  if (rejected.empty())
    return est;
  if (rejected.size() > cfg_.fde_max_frac * std::max(1, int(rows.size()))) {
    if (cp_hold_ == 0)
      cp_hold_ = cfg_.recov_cp_hold;
    return est;
  }
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
          st.release();
      }
    } else {
      current_.visible.insert({row.ref_sat, row.band});
      current_.visible.insert({row.target_sat, row.band});
    }
  }
  return est;
}
ArDecision Engine::resolve(Smoother &sm, const Values &estimate, Key pose,
                           const ObservationEpoch &obs) {
  std::map<int, const SatelliteObservation *> present;
  for (const auto &s : obs.sats)
    present[s.sat] = &s;
  std::vector<SF> keys;
  std::map<SF, double> floats;
  // Every eligible ambiguity, held or not, enters with its estimate and its
  // joint marginal covariance (a hold is a factor on the key, not a constant).
  std::map<SF, Key> amb;
  for (auto [sf, key] : sats_.keys())
    if (present.count(sf.first) && current_.visible.count(sf) &&
        !current_.seeded.count(sf) && estimate.exists(key)) {
      keys.push_back(sf);
      amb[sf] = key;
      floats[sf] = estimate.at<double>(key);
    }
  ArDecision out;
  if (keys.size() < 2)
    return out;
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
  std::map<std::pair<int, int>, std::vector<SF>> grouped;
  for (auto sf : keys)
    grouped[{present[sf.first]->system, sf.second}].push_back(sf);
  std::vector<std::pair<SF, SF>> pairs;
  for (auto &[group, members] : grouped) {
    std::sort(members.begin(), members.end());
    for (std::size_t i = 1; i < members.size(); ++i)
      pairs.push_back({members[0], members[i]});
  }
  if (pairs.size() < 2) {
    if (pairs.size() == 1)
      ar_outcome_ = ArOutcome::MinPairs;
    return out;
  }
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(pairs.size(), n);
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    D(i, index[pairs[i].first]) = 1;
    D(i, index[pairs[i].second]) = -1;
  }
  Eigen::VectorXd y = D * x;
  Eigen::MatrixXd Q = D * cov * D.transpose();
  auto res = gnss_fgo::ar::resolve(y, Q, 2, cfg_.ar_thresar);
  out.s0 = res.s0;
  out.s1 = res.s1;
  if (res.error)
    throw std::runtime_error("invalid integer-search covariance or reduction");
  if (res.s0 <= 1e-12 * std::max(res.s1, 1.))
    out.s0 = 0;
  if (!res.accepted)
    return out;
  std::set<int> used;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    auto [r, j] = pairs[i];
    used.insert(r.first);
    used.insert(j.first);
    out.fixed.emplace(r, floats[r]);
    out.fixed[j] = out.fixed[r] - res.integers[i];
    out.allowed.insert(r);
    out.allowed.insert(j);
    out.fixed_pairs.push_back(pairs[i]);
  }
  if (int(used.size()) < cfg_.ar_minfixsats)
    return ArDecision{};
  Eigen::VectorXd fixed = x;
  for (int i = 0; i < n; ++i)
    if (out.fixed.count(keys[i]))
      fixed[i] = out.fixed[keys[i]];
  Eigen::LLT<Eigen::MatrixXd> chol(Q);
  if (chol.info() != Eigen::Success)
    throw std::runtime_error("invalid conditional covariance");
  const Eigen::Vector3d correction =
      cross * D.transpose() * Q.partialPivLu().solve(D * (x - fixed));
  out.fixed_position = nav_position_ - R_ * correction;
  if (!out.fixed_position.allFinite())
    throw std::runtime_error("nonfinite conditional position");
  // Antenna position covariance conditioned on the fixed D x = integers:
  // P - C Q^-1 C', with C = cross D'.
  // Output only; it does not enter the estimate.
  {
    const gtsam::Matrix6 P6 = jm.at(pose, pose);
    Eigen::Matrix<double, 3, 6> J;
    J.leftCols<3>() = -rot * gtsam::skewSymmetric(lever);
    J.rightCols<3>() = rot;
    const Mat3 P = J * P6 * J.transpose();
    const Eigen::MatrixXd C = cross * D.transpose();
    Mat3 conditioned = P - C * chol.solve(C.transpose());
    conditioned = (.5 * (conditioned + conditioned.transpose())).eval();
    if (conditioned.allFinite() && (conditioned.diagonal().array() >= 0).all())
      out.fixed_covariance = conditioned;
  }
  out.nb = res.kept;
  return out;
}
ArDecision Engine::runAr(Smoother &sm, const Values &estimate, Key pose,
                         const ObservationEpoch &obs) {
  ArDecision result;
  ar_outcome_ = ArOutcome::NotCalled;
  if (!cfg_.ar_mode)
    return result;
  try {
    // One search over every eligible ambiguity: no subset retries.
    result = resolve(sm, estimate, pose, obs);
  } catch (const std::exception &) {
    return {};
  }
  if (result.nb) {
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
  if (!cfg_.gnss_motion && phase_ == 1)
    updateSwitchWindow(out);
  current_.finalize(sats_, out.fixed);
  ++input_epoch_;
  pruneImu();
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
  integrity_epoch_ = sanity_recovery_ = false;
  epoch_doppler_.reset();
  switch_reject_ = -1;
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
        auto dd = ddprPosition(obs, initial, stageLever());
        if (dd.position && std::isfinite(dd.rms) &&
            dd.rms <= cfg_.ddpr_max_res) {
          warmReset(*dd.position, initial.rotation());
          // The motion stage consumed this epoch's IMU at its start.
          if (!motionStage())
            collectImu(obs.gpst);
          bad_count_ = 0;
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
  out.b0 = stationaryBias(imu_, std::min<std::size_t>(3000, imu_index_),
                          cfg_.gravity);
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
  // min_dd_for_solve DD rows; not dead reckoning, not a reset or recovery
  // epoch), 2 posterior speed below vel_thresh, 3-6 the Doppler-only gate,
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
  // integrateImu); otherwise this epoch starts a new window.
  if (!window_.empty())
    try {
      integrateImu(cfg_, gtsam::imuBias::ConstantBias(), epoch_imu_,
                   window_.back().time, current_gpst_);
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
  // Forward horizontal axis of attitude(roll, pitch, 0): heading 0 = north,
  // the y axis of its ENU frame.
  for (std::size_t i = 1; i < window_.size(); ++i) {
    for (const auto &sample : window_[i].imu) {
      const Vec3 f = imu.rotation * (sample.acc - imu.b0.accelerometer()) -
                     Vec3(0, 0, cfg_.gravity);
      dv_imu += f.y() * (sample.gpst - previous);
      previous = sample.gpst;
    }
    // Interval end after the last sample (zero-order hold, as integrateImu).
    if (!window_[i].imu.empty()) {
      const Vec3 f =
          imu.rotation * (window_[i].imu.back().acc - imu.b0.accelerometer()) -
          Vec3(0, 0, cfg_.gravity);
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
  // Check 2: a sanity recovery in every GNSS epoch.
  if (sanity_recovery_)
    recovery_timer_.violate(current_gpst_, kIntegrityTime);
  else
    recovery_timer_.clear();
  // A timer whose last counting epoch is more than kIntegrityTime back has
  // lapsed (it restarts at its next violation).
  for (auto *timer : {&heading_timer_, &recovery_timer_})
    if (timer->last && current_gpst_ - *timer->last > kIntegrityTime)
      timer->clear();
  if (clean && !sanity_recovery_)
    good_bias_ = bias_;
  if (heading_timer_.length() >= kIntegrityTime - eps)
    return 1;
  if (recovery_timer_.length() >= kIntegrityTime - eps)
    return 2;
  return 0;
}
Solution Engine::fallback(const ObservationEpoch &obs, bool gnss, int cause) {
  ++n_fallback_;
  fallback_cause_ = cause;
  // The next switch levels with the same roll/pitch; its bias starts from
  // the last estimate of this graph while both checks were clean, if any.
  LevelImu level = levelImu();
  if (good_bias_)
    level.b0 = *good_bias_;
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
  zupt_anchor_.reset();
  per_sat_res_.clear();
  cp_hold_ = cfg_.recov_cp_hold;
  bad_count_ = 0;
  pim_break_ = false;
  healthy_ = true;
  window_.clear();
  heading_timer_.clear();
  recovery_timer_.clear();
  // This epoch is solved again by the new motion graph (its start path: the
  // DD code position as the seed).
  current_ = EpochState{};
  motion_update_ = integrity_epoch_ = sanity_recovery_ = false;
  epoch_doppler_.reset();
  return phase2(obs, gnss);
}
void Engine::transition(bool reverse) {
  const auto saved_sats = sats_;
  const auto saved_epoch = current_;
  try {
    // Initial bias and roll/pitch (levelImu: from the session's first
    // samples, or what an earlier switch used, with the bias of a fallback).
    const LevelImu level = levelImu();
    if (!level.level)
      throw std::runtime_error("initialization needs IMU");
    const auto b0 = level.b0;
    const double pitch = level.pitch, roll = level.roll;
    const int n = window_.size();
    if (n < 2)
      throw std::logic_error("switch window too short");
    std::vector<Vec3> enu;
    for (const auto &w : window_)
      enu.push_back(R_.transpose() * (w.position - base_));
    // Preintegration of every window interval (pims[i - 1]: epoch i - 1 to i).
    std::vector<IntegratedImu> pims;
    for (int i = 1; i < n; ++i)
      pims.push_back(integrateImu(cfg_, b0, window_[i].imu,
                                  window_[i - 1].time, window_[i].time));
    // Heading = direction of the horizontal posterior antenna velocity (plus
    // 180 deg when reversing); the window mean velocity where one is too
    // slow.
    Vec3 mean_velocity = Vec3::Zero();
    for (const auto &w : window_)
      mean_velocity += w.velocity / n;
    double previous_heading = std::atan2(mean_velocity.x(), mean_velocity.y());
    const double rstd = cfg_.body_rot_std * M_PI / 180.,
                 min_yaw = 2. * M_PI / 180.;
    Graph graph;
    Values values;
    gtsam::FixedLagSmoother::KeyTimestampMap ts;
    for (int i = 0; i < n; ++i) {
      const auto &w = window_[i];
      const double speed = w.velocity.head<2>().norm();
      const double heading = speed > .01
                                 ? std::atan2(w.velocity.x(), w.velocity.y())
                                 : previous_heading;
      previous_heading = heading;
      // Reversing: the body x axis points against the velocity.
      const double body_heading = reverse ? heading + M_PI : heading;
      const Mat3 rot = attitude(roll, pitch, body_heading);
      // IMU-origin velocity: the antenna velocity minus the lever-arm
      // rotation term, with the bias-corrected mean angular rate of the
      // adjacent window interval.
      const Vec3 omega = pims[i == 0 ? 0 : i - 1].mean_gyro - b0.gyroscope();
      const Vec3 velocity = w.velocity - rot * omega.cross(lever_);
      gtsam::Pose3 pose(gtsam::Rot3(rot), enu[i] - rot * lever_);
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
      graph.addPrior<gtsam::imuBias::ConstantBias>(biasKey(i), b0,
                                                   noise(6, .01));
      if (i > 0)
        graph.emplace_shared<gtsam::CombinedImuFactor>(
            poseKey(i - 1), velKey(i - 1), poseKey(i), velKey(i),
            biasKey(i - 1), biasKey(i), pims[i - 1].pim);
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
    bad_count_ = 0;
    zupt_anchor_.reset();
    pim_break_ = false;
    window_.clear();
    saved_level_ = level;
    heading_timer_.clear();
    recovery_timer_.clear();
  } catch (...) {
    sats_ = saved_sats;
    current_ = saved_epoch;
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
  graph.emplace_shared<gtsam::CombinedImuFactor>(
      poseKey(k - 1), velKey(k - 1), poseKey(k), velKey(k), biasKey(k - 1),
      biasKey(k), im.pim);
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
      biasKey(k - 1), biasKey(k), gtsam::imuBias::ConstantBias(),
      splitNoise(cfg_.bias_between_acc_sigma, cfg_.bias_between_gyro_sigma));
  if (cfg_.bias_prior_mode)
    graph.addPrior<gtsam::imuBias::ConstantBias>(
        biasKey(k), cfg_.bias_prior_mode == 1 ? initial_bias_ : bias_,
        splitNoise(cfg_.bias_prior_acc_sigma, cfg_.bias_prior_gyro_sigma));
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
void Engine::addPseudo(Graph &graph, int k, const gtsam::Pose3 &previous,
                       const Vec3 &velocity,
                       const std::vector<ImuSample> &samples,
                       const Vec3 &gyro) {
  const double speed = velocity.head<2>().norm();
  if (cfg_.nhc_enable && speed >= cfg_.nhc_min_speed) {
    Vec3 lever = Vec3::Zero();
    std::string text = cfg_.nhc_lever;
    std::replace(text.begin(), text.end(), ',', ' ');
    std::istringstream stream(text);
    stream >> lever.x() >> lever.y() >> lever.z();
    graph.emplace_shared<NhcFactor>(
        poseKey(k), velKey(k), gyro - bias_.gyroscope(), lever, speed, cfg_);
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
  stationary = stationary && velocity.norm() < kZuptMaxSpeed;
  if (!stationary) {
    zupt_anchor_.reset();
    return;
  }
  graph.addPrior<gtsam::Vector3>(velKey(k), Vec3::Zero(),
                                 noise(3, cfg_.zupt_sigma_zero_velocity));
  if (k > 0)
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        poseKey(k - 1), poseKey(k), gtsam::Pose3(),
        splitNoise(cfg_.zupt_sigma_zero_rotation, 1e3));
  if (!zupt_anchor_)
    zupt_anchor_ = previous;
  else
    graph.addPrior<gtsam::Pose3>(
        poseKey(k), *zupt_anchor_,
        splitNoise(cfg_.zupt_anchor_sigma_rotation,
                   cfg_.zupt_anchor_sigma_translation));
}
std::optional<Engine::DopplerRows>
Engine::dopplerRows(const ObservationEpoch &obs, const gtsam::NavState &pred) {
  std::vector<DopplerRow> rows;
  for (const auto &s : obs.sats) {
    if (s.system == 4)
      continue;
    for (int f = 0; f < 3; ++f)
      if (s.doppler[f] && s.doppler_wavelength[f] > 0) {
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
std::pair<double, std::map<int, double>>
Engine::residuals(const Graph &graph, const DdRows &rows,
                  const Values &values) const {
  double sum = 0;
  int n = 0;
  std::map<int, double> per_sat;
  for (const auto &row : rows) {
    if (row.carrier)
      continue;
    double r =
        std::sqrt(2. * std::max(0., graph.at(row.index)->error(values))) *
        cfg_.sigma_pr * std::sqrt(2.);
    sum += r * r;
    ++n;
    for (int sat : {row.ref_sat, row.target_sat})
      if (r > per_sat[sat])
        per_sat[sat] = r;
  }
  return {n ? std::sqrt(sum / n) : 0., per_sat};
}
void Engine::resetAmbiguities() {
  std::set<Key> keys;
  for (auto [sf, key] : sats_.keys())
    keys.insert(key);
  gtsam::FactorIndices remove;
  // The smoother's marginal factor of the epochs that left the lag (on the
  // oldest state and the ambiguity keys of that time) is the only information
  // the window's states have from before it; without it they rest on the
  // window's code DDs alone, indeterminate with fewer than 3 independent ones.
  // It is replaced by its marginal on the states (ambiguities integrated out).
  Graph marginals;
  const auto &graph = p2_->getFactors();
  for (std::size_t i = 0; i < graph.size(); ++i)
    if (graph[i])
      for (auto key : graph[i]->keys())
        if (keys.count(key)) {
          remove.push_back(i);
          if (const auto *marginal =
                  dynamic_cast<const gtsam::LinearContainerFactor *>(
                      graph[i].get()))
            if (auto kept = marginalWithout(*marginal, keys))
              marginals.push_back(kept);
          break;
        }
  try {
    if (!remove.empty()) {
      updateProbe("reset_ambiguities", false);
      p2_->update(marginals, Values{},
                  gtsam::FixedLagSmoother::KeyTimestampMap{}, remove);
      updateProbe("reset_ambiguities", true);
    }
  } catch (...) {
    healthy_ = false;
    throw;
  }
  // The removed factors leave these keys unused; ISAM2 drops them and the
  // smoother their timestamps. The new arcs get new keys.
  holds_.clear();
  for (auto &[sf, s] : sats_.track)
    s.invalidate();
  current_.accepted.clear();
  cp_hold_ = cfg_.recov_cp_hold;
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
    zupt_anchor_.reset();
    per_sat_res_.clear();
    cp_hold_ = cfg_.recov_cp_hold;
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
  zupt_anchor_.reset();
  per_sat_res_.clear();
  current_.accepted.clear();
  current_.visible.clear();
  cp_hold_ = cfg_.recov_cp_hold;
  pim_break_ = true;
  nav_position_ = antenna(pose);
}
std::optional<Solution> Engine::sanity(const Graph &graph, const DdRows &rows,
                                       const gtsam::Pose3 &pose,
                                       const gtsam::NavState &pred, int k) {
  if (main_res_ <= cfg_.main_ddpr_res_thresh) {
    bad_count_ = 0;
    return {};
  }
  std::vector<double> r;
  for (auto [s, v] : ar_per_sat_res_)
    r.push_back(v);
  std::sort(r.begin(), r.end());
  if (int(r.size()) >= cfg_.sanity_max_median_min_sats &&
      r[r.size() / 2] > 1e-3 &&
      r.back() / r[r.size() / 2] > cfg_.sanity_max_median_ratio)
    return {};
  Values p;
  p.insert(poseKey(k), pred.pose());
  double pred_res = residuals(graph, rows, p).first;
  bool fast = main_res_ > cfg_.main_ddpr_res_catastrophic && !r.empty() &&
              r.back() >= cfg_.ddpr_fast_worst_sat_min;
  if (!fast) {
    ++bad_count_;
    cp_hold_ = std::max(cp_hold_, cfg_.recov_cp_hold);
    if (bad_count_ < cfg_.ddpr_sanity_persist)
      return {};
  }
  resetAmbiguities();
  bad_count_ = 0;
  const bool replace = pred_res <= cfg_.sanity_pose_replace_thresh &&
                       (pose.translation() - pred.pose().translation()).norm() >
                           cfg_.sanity_pose_replace_thresh;
  Solution sol;
  sol.position = antenna(replace ? pred.pose() : pose);
  sol.valid = true;
  sol.kind = replace ? UpdateKind::DeadReckoning : UpdateKind::Gnss;
  nav_position_ = antenna(pose);
  return sol;
}
// One key per ambiguity arc: a key carries over from epoch to epoch with no
// new prior; the carrier DDs of all its epochs share it.
// An arc ends (the next carrier DD starts a new key) on:
//  - a slip or outage: LLI, input slip, geometry-free jump, or 6 epochs in a
//    row without the band's carrier on both receivers (slips(); process()
//    for epochs without GNSS);
//  - a CP-suppressed epoch or a key the smoother no longer has
//    (continueArcs());
//  - a GNSS epoch without a carrier DD of the satellite/band (endUnusedArcs());
//  - a hold release: after a GNSS epoch without a fix, the held band with the
//    largest residual above a threshold (phase2()); the holds a rejected
//    carrier DD contradicts (fde());
//  - an ambiguity reset (resetAmbiguities(), warmReset(), fallback()) or the
//    switch to the IMU graph (transition()), which end every arc.
void Engine::continueArcs(const Values &estimate) {
  for (auto &[sf, s] : sats_.track) {
    if (s.key && (current_.cp_suppressed || !estimate.exists(*s.key)))
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
  Values estimate;
  gtsam::Pose3 previous;
  Vec3 velocity = Vec3::Zero();
  gtsam::imuBias::ConstantBias previous_bias;
  gtsam::NavState pred;
  Graph graph;
  Values values;
  std::vector<ImuSample> samples;
  IntegratedImu integrated(cfg_, bias_);
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
    const auto cursor = imu_index_;
    samples = collectImu(obs.gpst);
    try {
      integrated = integrateImu(
          cfg_, bias_, samples,
          graph_gpst_.value_or(std::numeric_limits<double>::quiet_NaN()),
          obs.gpst);
    } catch (...) {
      imu_index_ = cursor;
      throw;
    }
    estimate = p2_->calculateEstimate();
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
    current_.cp_suppressed = cp_hold_ > 0;
    if (current_.cp_suppressed)
      --cp_hold_;
    continueArcs(estimate);
    nv = buildDd(graph, rows, values, obs, poseKey(k), stageLever(),
                 current_.cp_suppressed);
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
  if (!motionStage())
    addPseudo(graph, k, previous, velocity, samples, integrated.mean_gyro);
  // Every key of a live arc is refreshed, also in epochs without GNSS; an
  // ended arc's key is not and leaves through the lag.
  std::vector<Key> keep;
  for (auto [sf, key] : sats_.keys())
    keep.push_back(key);
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
      bad_count_ = 0;
      out.position = nav_position_;
      out.valid = true;
      out.kind = UpdateKind::Gnss;
    }
    return out;
  }
  // Satellites with a code DD residual, in the order of the rows.
  std::vector<int> residual_sat_order;
  if (gnss) {
    auto res = residuals(graph, rows, estimate);
    main_res_ = res.first;
    ar_per_sat_res_ = res.second;
    std::set<int> seen;
    for (const auto &row : rows) {
      if (row.carrier)
        continue;
      for (int s : {row.ref_sat, row.target_sat})
        if (ar_per_sat_res_.count(s) && seen.insert(s).second)
          residual_sat_order.push_back(s);
    }
    estimate = fde(*p2_, graph, rows, slots, estimate, poseKey(k));
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
  ArDecision ar;
  if (gnss && k >= (motionStage() ? 5 : cfg_.n_collect + 3) &&
      !current_.cp_suppressed && !cp_hold_) {
    double gdop = 999.;
    if (obs.sats.size() >= 4) {
      Eigen::MatrixXd G(obs.sats.size(), 4);
      const Vec3 pred_body = base_ + R_ * pred.pose().translation();
      for (std::size_t i = 0; i < obs.sats.size(); ++i) {
        G.block<1, 3>(i, 0) =
            (obs.sats[i].rover_sat - pred_body).normalized().transpose();
        G(i, 3) = 1;
      }
      Eigen::Matrix4d Q = (G.transpose() * G).inverse();
      gdop = std::sqrt(Q.trace());
    }
    if (gdop <= cfg_.ar_gdop_max)
      ar = runAr(*p2_, estimate, poseKey(k), obs);
  }
  if (gnss && !ar.nb) {
    auto recovery = sanity(graph, rows, pose, pred, k);
    if (recovery) {
      sanity_recovery_ = integrity_epoch_ = !motionStage();
      return *recovery;
    }
  }
  if (ar.nb)
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
      sats_.get({std::get<2>(*best), std::get<3>(*best)}).release();
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
  return sol;
}
} // namespace gnss_fgo
