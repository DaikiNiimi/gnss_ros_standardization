#include "engine.hpp"
#include "ambiguity.hpp"
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
auto isotropic(int n, double sigma) {
  return gtsam::noiseModel::Isotropic::Sigma(n, sigma);
}
auto diag6(double r1, double r2, double r3, double t1, double t2, double t3) {
  gtsam::Vector6 s;
  s << r1, r2, r3, t1, t2, t3;
  return gtsam::noiseModel::Diagonal::Sigmas(s);
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
std::unique_ptr<Engine::Smoother> Engine::makeSmoother(double lag,
                                                       const Config &c) {
  c.validate();
  gtsam::ISAM2Params p;
  p.relinearizeSkip = c.isam2_relinearize_skip;
  p.relinearizeThreshold = c.isam2_relinearize_threshold;
  // Reuse the slots of removed and marginalized factors; otherwise the factor
  // vector grows by every factor ever added (unbounded over a long run).
  // New factors are addressed through ISAM2Result::newFactorsIndices.
  p.findUnusedFactorSlots = true;
  if (c.gnss_motion) p.factorization = gtsam::ISAM2Params::QR;
  return std::make_unique<Smoother>(lag, p);
}
Engine::Engine(const Config &c, const Vec3 &initial, const Vec3 &base,
               const Vec3 &lever)
    : cfg_(c), base_(base), lever_(lever), nav_position_(initial),
      R_(enuRotation(base)), frame_(gtsam::Rot3(R_), base),
      p1_(makeSmoother(c.phase1_fls_lag, c)) {
  if (!initial.allFinite() || !base.allFinite() || !lever.allFinite())
    throw std::invalid_argument("nonfinite initial coordinates");
  if (cfg_.gnss_motion) {
    if (lever.norm() != 0) throw std::invalid_argument("GNSS-only state is the antenna; no body lever arm");
    phase_ = 2;
    graph_epoch_ = -1;
    actual_lag_ = cfg_.fls_lag;
    p2_ = makeSmoother(actual_lag_, cfg_);
  }
}
void Engine::addImu(const ImuSample &s) {
  if (cfg_.gnss_motion) throw std::invalid_argument("IMU input is not part of the GNSS-only model");
  if (!std::isfinite(s.gpst) || (!imu_.empty() && s.gpst <= imu_.back().gpst))
    throw std::invalid_argument("unordered IMU input");
  auto sample = s;
  if (!imu_.empty())
    sample.predecessor = imu_.back().gpst;
  imu_.push_back(sample);
  // Bounded backlog: without GNSS epochs nothing consumes IMU samples. Drop the
  // oldest unconsumed one (counted); the gap it leaves fails the next interval
  // check, which resets the motion state instead of integrating across it.
  if (imu_.size() - imu_index_ > kImuBacklog) {
    imu_.erase(imu_.begin() + imu_index_);
    ++imu_backlog_dropped_;
  }
}
void Engine::pruneImu() {
  // Consumed samples are needed only for the initialization statistics (the
  // first kInitSamples, while still in phase 1) and the last consumed sample
  // (output angular rate). Erase the rest in batches.
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
  auto prev = current_.references.find(sys);
  for (int i : idx)
    if (prev != current_.references.end() && obs.sats[i].sat == prev->second &&
        !geo(obs.sats[i]) &&
        per_sat_res_[prev->second] <= cfg_.per_sat_res_thresh)
      return i;
  std::vector<int> pool;
  for (int i : idx)
    if (!geo(obs.sats[i]) &&
        per_sat_res_[obs.sats[i].sat] <= cfg_.per_sat_res_thresh)
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
int Engine::buildDd(Graph &graph, Values &values, const ObservationEpoch &obs,
                    Key pose, const Vec3 &lever, int dd_epoch,
                    const std::map<SF, std::pair<Key, double>> &previous,
                    bool skip_cp) {
  current_.visible.clear();
  int nv = 0;
  for (const auto &[sys, idx] : groups(obs)) {
    if (idx.size() < 2)
      continue;
    const int ri = reference(obs, idx);
    const auto &r = obs.sats[ri];
    current_.references[sys] = r.sat;
    for (int ji : idx) {
      if (ji == ri)
        continue;
      const auto &j = obs.sats[ji];
      if (cfg_.exclude_bds_geo && geo(j))
        continue;
      for (int f = 0; f < 3; ++f) {
        if (!r.pr[f] || !r.pr_base[f] || !j.pr[f] || !j.pr_base[f])
          continue;
        const double pr = (r.pr[f] - j.pr[f]) - (r.pr_base[f] - j.pr_base[f]);
        graph.emplace_shared<DdFactor>(
            pose, std::vector<Key>{}, std::vector<double>{}, r, j, f, base_,
            lever, frame_, pr, 0.,
            ddSigma(cfg_, true, r.elevation, j.elevation,
                    obs.gpst - obs.base_gpst),
            false);
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
          if (st.held || st.key)
            continue;
          Key key = ambKey(sf, dd_epoch * 100 + st.generation);
          double n = ((sat->phase[f] * lam - sat->phase_base[f] * lam) -
                      (sat->pr[f] - sat->pr_base[f])) /
                     lam;
          double sigma = phase_ == 2 ? cfg_.sigma_amb0 : 3.;
          auto old = previous.find(sf);
          if (st.seed_pending && st.last_held) {
            n = *st.last_held;
            sigma = .1;
            current_.seeded.insert(sf);
            current_.consumed.insert(sf);
          } else if (old != previous.end()) {
            n = old->second.second;
            sigma = cfg_.sigma_cont;
          } else
            current_.seeded.insert(sf);
          if (!values.exists(key)) {
            values.insert(key, n);
            graph.addPrior<double>(key, n, isotropic(1, sigma));
          }
          st.key = key;
        }
        std::vector<Key> keys;
        std::vector<double> coeff;
        double offset = 0;
        for (const auto &item : std::vector<std::pair<SF, double>>{
                 {{r.sat, f}, lam}, {{j.sat, f}, -lam}}) {
          auto &st = sats_.get(item.first);
          if (st.held)
            offset += item.second * *st.held;
          else {
            keys.push_back(*st.key);
            coeff.push_back(item.second);
          }
        }
        const double cp = (r.phase[f] * lam - j.phase[f] * lam) -
                          (r.phase_base[f] * lam - j.phase_base[f] * lam);
        graph.emplace_shared<DdFactor>(
            pose, keys, coeff, r, j, f, base_, lever, frame_, cp, offset,
            ddSigma(cfg_, false, r.elevation, j.elevation,
                    obs.gpst - obs.base_gpst),
            true);
        ++nv;
      }
    }
  }
  return nv;
}
DdPosition Engine::ddprPosition(const ObservationEpoch &obs,
                                const gtsam::Pose3 &initial,
                                const Vec3 &lever) {
  std::vector<std::shared_ptr<DdFactor>> factors;
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
        factors.push_back(std::make_shared<DdFactor>(
            key, std::vector<Key>{}, std::vector<double>{}, r, j, f, base_,
            lever, frame_, (r.pr[f] - j.pr[f]) - (r.pr_base[f] - j.pr_base[f]),
            0., cfg_.sigma_pr * std::sqrt(2.), false));
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
    graph.addPrior<gtsam::Pose3>(key, initial, diag6(.05, .05, .1, 50, 50, 50));
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
Engine::Values Engine::fde(Smoother &sm, const Graph &graph,
                           const gtsam::FactorIndices &slots, Values est,
                           Key pose, int nv) {
  if (slots.size() != graph.size())
    throw std::logic_error("factor slots do not match the update graph");
  if (!cfg_.fde_enable)
    return est;
  gtsam::FactorIndices rejected;
  int total = 0;
  for (std::size_t i = 0; i < graph.size(); ++i) {
    auto f = std::dynamic_pointer_cast<DdFactor>(graph[i]);
    if (!f || (f->carrier && f->coefficients.empty()))
      continue;
    ++total;
    double res = std::sqrt(2 * f->error(est)) *
                 (f->carrier ? cfg_.sigma_cp : cfg_.sigma_pr) * std::sqrt(2.);
    if (res > (f->carrier ? cfg_.fde_cp : cfg_.fde_pr))
      rejected.push_back(slots[i]);
  }
  if (nv < 0)
    nv = total;
  if (rejected.empty())
    return est;
  if (rejected.size() > cfg_.fde_max_frac * std::max(1, nv)) {
    if (phase_ == 2 && cp_hold_ == 0)
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
  for (std::size_t i = 0; i < graph.size(); ++i) {
    auto f = std::dynamic_pointer_cast<DdFactor>(graph[i]);
    if (!f || !f->carrier)
      continue;
    if (removed.count(slots[i])) {
      if (f->coefficients.size() == 1)
        for (int sat : {f->ref_sat, f->target_sat}) {
          auto &st = sats_.get({sat, f->band});
          if (st.held)
            st.release();
        }
    } else {
      current_.visible.insert({f->ref_sat, f->band});
      current_.visible.insert({f->target_sat, f->band});
    }
  }
  return est;
}
ArDecision Engine::resolve(Smoother &sm, const Values &estimate, Key pose,
                           const ObservationEpoch &obs,
                           const std::set<int> &exclude, bool partial,
                           std::optional<gtsam::JointMarginal> *covariance_cache) {
  std::map<int, const SatelliteObservation *> present;
  for (const auto &s : obs.sats)
    if (!exclude.count(s.sat))
      present[s.sat] = &s;
  std::vector<SF> keys;
  std::map<SF, double> floats;
  auto eligible = [&](SF sf) {
    return present.count(sf.first) && current_.visible.count(sf);
  };
  for (auto sf : sats_.insertion_order) {
    auto &st = sats_.get(sf);
    if (st.held && eligible(sf)) {
      keys.push_back(sf);
      floats[sf] = *st.held;
    }
  }
  for (auto [sf, key] : sats_.keys())
    if (!floats.count(sf) && eligible(sf) && !current_.seeded.count(sf) &&
        estimate.exists(key)) {
      keys.push_back(sf);
      floats[sf] = estimate.at<double>(key);
    }
  ArDecision out;
  if (keys.size() < 2) {
    ar_outcome_ = ArOutcome::Unposed;
    return out;
  }
  gtsam::KeyVector query{pose};
  std::vector<SF> in_graph;
  for (auto sf : keys) {
    auto &st = sats_.get(sf);
    if (st.key && estimate.exists(*st.key)) {
      query.push_back(*st.key);
      in_graph.push_back(sf);
    }
  }
  // All subset attempts in one runAr see the same posterior. Its first
  // query contains every eligible key, so later attempts use sub-blocks of
  // that joint marginal instead of repeating Bayes-tree covariance queries.
  // The cache is local to runAr and cannot survive a graph update/reset.
  std::optional<gtsam::JointMarginal> local_marginal;
  auto &marginal = covariance_cache ? *covariance_cache : local_marginal;
  if (!marginal)
    marginal.emplace(sm.getISAM2().jointMarginalCovariance(query));
  const auto &jm = *marginal;
  int n = keys.size();
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(n, n),
                  cross = Eigen::MatrixXd::Zero(3, n);
  Eigen::VectorXd x(n);
  std::map<SF, int> index;
  const Mat3 rot = estimate.at<gtsam::Pose3>(pose).rotation().matrix();
  const Vec3 lever = phase_ == 2 ? lever_ : Vec3::Zero();
  for (int i = 0; i < n; ++i) {
    auto sf = keys[i];
    index[sf] = i;
    x[i] = floats[sf];
    auto &st = sats_.get(sf);
    if (st.key && estimate.exists(*st.key)) {
      for (int j = 0; j < n; ++j) {
        auto &sj = sats_.get(keys[j]);
        if (sj.key && estimate.exists(*sj.key))
          cov(i, j) = jm.at(*st.key, *sj.key)(0, 0);
      }
      const gtsam::Vector6 block = jm.at(pose, *st.key).col(0);
      cross.col(i) = rot * (block.tail<3>() - lever.cross(block.head<3>()));
    }
    if (st.held)
      cov(i, i) = std::max(cfg_.varholdamb, 1e-6);
  }
  std::map<std::pair<int, int>, std::vector<SF>> grouped;
  for (auto sf : keys)
    if (present[sf.first]->elevation >= 20 * M_PI / 180.)
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
  auto res = gnss_fgo::ar::resolve(y, Q, false, partial, 2);
  out.s0 = res.s0;
  out.s1 = res.s1;
  if (res.error)
    throw std::runtime_error("invalid integer-search covariance or reduction");
  if (ar_observer_)
    ar_observer_(input_epoch_, partial, y, Q, res.accepted ? res.kept : 0,
                 res.s0, res.s1, res.dropped);
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
    out.fixed[j] = out.fixed[r] - res.conditioned[i];
    if (!res.dropped || res.exact_rows[i]) {
      out.allowed.insert(r);
      out.allowed.insert(j);
    }
  }
  if (int(used.size()) < cfg_.ar_minfixsats) {
    ar_outcome_ = ArOutcome::MinSatellites;
    return ArDecision{};
  }
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
  // Antenna position covariance conditioned on the fixed combinations
  // A * (D x) = integers: P - C A' (A Q A')^-1 A C', with C = cross D'.
  // Output only; it does not enter the estimate.
  {
    const gtsam::Matrix6 P6 = jm.at(pose, pose);
    Eigen::Matrix<double, 3, 6> J;
    J.leftCols<3>() = -rot * gtsam::skewSymmetric(lever);
    J.rightCols<3>() = rot;
    const Mat3 P = J * P6 * J.transpose();
    const Eigen::MatrixXd C = cross * D.transpose() * res.A.transpose();
    const Eigen::MatrixXd S = res.A * Q * res.A.transpose();
    Eigen::LLT<Eigen::MatrixXd> s_chol(S);
    if (s_chol.info() == Eigen::Success) {
      Mat3 conditioned = P - C * s_chol.solve(C.transpose());
      conditioned = (.5 * (conditioned + conditioned.transpose())).eval();
      if (conditioned.allFinite() && (conditioned.diagonal().array() >= 0).all())
        out.fixed_covariance = conditioned;
    }
  }
  out.nb = res.kept;
  out.dropped = res.dropped;
  return out;
}
ArDecision Engine::runAr(Smoother &sm, const Values &estimate, Key pose,
                         const ObservationEpoch &obs) {
  std::optional<gtsam::JointMarginal> covariance;
  auto solve = [&](const std::set<int> &exclude = std::set<int>{},
                   bool partial = false) {
    return resolve(sm, estimate, pose, obs, exclude, partial,
                   cfg_.gnss_motion ? &covariance : nullptr);
  };
  ArDecision result;
  ar_outcome_ = ArOutcome::Entered;
  if (!cfg_.ar_mode) {
    ar_outcome_ = ArOutcome::NotCalled;
    return result;
  }
  try {
    result = solve();
  } catch (const std::exception &) {
    ar_outcome_ = ArOutcome::Exception;
    return {};
  }
  try {
    if (!result.nb && cfg_.subset_ar_enable &&
        int(sats_.keys().size()) >= cfg_.subset_ar_min_nb + 1) {
      int dirty = 0;
      for (auto [s, res] : ar_per_sat_res_)
        if (res > cfg_.subset_ar_dirty_sat_res_m)
          ++dirty;
      if (cfg_.subset_ar_max_dirty_sats <= 0 ||
          dirty <= cfg_.subset_ar_max_dirty_sats) {
        std::map<int, double> elevations;
        for (auto &s : obs.sats)
          elevations[s.sat] = std::max(elevations[s.sat], s.elevation);
        std::set<int> seen;
        std::vector<std::tuple<double, double, int>> ranked;
        for (auto [sf, key] : sats_.keys())
          if (seen.insert(sf.first).second)
            ranked.emplace_back(ar_per_sat_res_[sf.first],
                                -elevations[sf.first], sf.first);
        std::sort(ranked.rbegin(), ranked.rend());
        if (int(ranked.size()) > cfg_.subset_ar_max_candidates)
          ranked.resize(cfg_.subset_ar_max_candidates);
        std::optional<std::tuple<double, int, int>> best;
        std::set<int> best_drop;
        std::function<void(int, int, std::set<int> &)> visit =
            [&](int begin, int remaining, std::set<int> &drop) {
              if (!remaining) {
                auto a = solve(drop);
                if (a.nb < cfg_.subset_ar_min_nb)
                  return;
                double ratio = a.s0 > 0 ? a.s1 / a.s0 : 0.;
                auto score = std::make_tuple(std::nearbyint(ratio * 1e6) / 1e6,
                                             a.nb, -int(drop.size()));
                if (!best || score > *best) {
                  best = score;
                  best_drop = drop;
                }
                return;
              }
              for (int i = begin; i < int(ranked.size()); ++i) {
                drop.insert(std::get<2>(ranked[i]));
                visit(i + 1, remaining - 1, drop);
                drop.erase(std::get<2>(ranked[i]));
              }
            };
        for (int k = 1; k <= cfg_.subset_ar_max_drop; ++k) {
          std::set<int> drop;
          visit(0, k, drop);
        }
        if (best)
          result = solve(best_drop);
      }
    }
  } catch (const std::exception &) {
    result = {};
  }
  if (!result.nb) {
    try {
      result = solve({}, true);
    } catch (const std::exception &) {
      ar_outcome_ = ArOutcome::Exception;
      result = {};
    }
  }
  if (result.nb) {
    ar_outcome_ = ArOutcome::Success;
    for (auto sf : result.allowed)
      current_.accepted[sf] = sats_.get(sf).arc;
  } else if (ar_outcome_ != ArOutcome::MinPairs)
    ar_outcome_ = ArOutcome::Zero;
  return result;
}
void Engine::hold(Smoother &sm, const ArDecision &ar) {
  if (!ar.nb)
    return;
  Graph graph;
  gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
  for (auto sf : ar.allowed) {
    auto &st = sats_.get(sf);
    if (!st.key)
      continue;
    if (phase_ == 1) {
      graph.addPrior<double>(*st.key, ar.fixed.at(sf),
                             isotropic(1, std::sqrt(cfg_.varholdamb)));
      timestamps[*st.key] = phase1_time_;
    }
  }
  if (!graph.empty())
    try {
      updateProbe("hold", false);
      sm.update(graph, Values{}, timestamps);
      updateProbe("hold", true);
    } catch (...) {
      healthy_ = false;
      throw;
    }
  for (auto sf : ar.allowed) {
    auto &st = sats_.get(sf);
    if (phase_ == 2 && st.key) {
      st.activate(ar.fixed.at(sf));
      st.key.reset();
    }
    st.fix_flag = 3;
  }
}
void Engine::restartPhase1() {
  p1_ = makeSmoother(cfg_.phase1_fls_lag, cfg_);
  p1_inserted_.clear();
  healthy_ = true;
  collection_.clear();
  fix_history_.clear();
  collecting_ = false;
  zupt_anchor_.reset();
  per_sat_res_.clear();
  current_.accepted.clear();
  current_.visible.clear();
  for (auto &[sf, s] : sats_.track)
    s.invalidate();
}
Solution Engine::phase1(const ObservationEpoch &obs) {
  slips(obs);
  const Key key = xp(input_epoch_);
  const bool fresh =
      input_epoch_ == 0 || !p1_inserted_.count(xp(input_epoch_ - 1));
  gtsam::Pose3 initial(gtsam::Rot3(), R_.transpose() * (nav_position_ - base_));
  if (fresh)
    try {
      auto dd = ddprPosition(obs, initial, Vec3::Zero());
      if (dd.position && dd.active >= 4 && std::isfinite(dd.rms)) {
        nav_position_ = *dd.position;
        initial = gtsam::Pose3(gtsam::Rot3(),
                               R_.transpose() * (nav_position_ - base_));
      }
    } catch (const std::exception &) {
    }
  phase1_time_ += dt_;
  Values est;
  for (int attempt = 0; attempt < 2; ++attempt) {
    Graph graph;
    Values values;
    values.insert(key, initial);
    if (fresh || attempt)
      graph.addPrior<gtsam::Pose3>(key, initial, diag6(.01, .01, .01, 3, 3, 3));
    else
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          xp(input_epoch_ - 1), key, gtsam::Pose3(),
          diag6(.001, .001, .001, .1, .1, .1));
    buildDd(graph, values, obs, key, Vec3::Zero());
    for (auto k : values.keys())
      if (p1_inserted_.count(k))
        values.erase(k);
    gtsam::FixedLagSmoother::KeyTimestampMap ts;
    ts[key] = phase1_time_;
    if (!fresh && !attempt)
      ts[xp(input_epoch_ - 1)] = phase1_time_;
    for (auto k : values.keys())
      ts[k] = phase1_time_;
    for (auto [sf, k] : sats_.keys())
      ts[k] = phase1_time_;
    gtsam::FactorIndices slots;
    try {
      p1_->update(graph, values, ts);
      slots = p1_->getISAM2Result().newFactorsIndices;
      for (auto sf : current_.consumed)
        sats_.get(sf).seed_pending = false;
    } catch (const std::exception &) {
      restartPhase1();
      if (!attempt)
        continue;
      return {};
    }
    p1_inserted_.insert(key);
    for (auto k : values.keys())
      p1_inserted_.insert(k);
    try {
      for (int j = 0; j < 3; ++j)
        p1_->update(Graph{}, Values{}, {});
      est = p1_->calculateEstimate();
    } catch (const std::exception &) {
      restartPhase1();
      return {};
    }
    for (auto it = p1_inserted_.begin(); it != p1_inserted_.end();)
      if (!est.exists(*it))
        it = p1_inserted_.erase(it);
      else
        ++it;
    est = fde(*p1_, graph, slots, est, key);
    break;
  }
  const auto pose = est.at<gtsam::Pose3>(key);
  if (!pose.matrix().allFinite())
    throw std::runtime_error("nonfinite phase1 pose");
  nav_position_ = R_ * pose.translation() + base_;
  ArDecision ar;
  if (input_epoch_ >= 5)
    ar = runAr(*p1_, est, key, obs);
  if (ar.nb)
    hold(*p1_, ar);
  Solution out;
  out.position = ar.nb ? ar.fixed_position : nav_position_;
  if (ar.nb)
    out.position_covariance = R_ * ar.fixed_covariance * R_.transpose();
  out.valid = true;
  out.kind = UpdateKind::Gnss;
  out.fixed = ar.nb > 0;
  out.nb = ar.nb;
  out.ratio = ar.s0 > 0 ? ar.s1 / ar.s0 : 0;
  collect(obs, out);
  return out;
}
Solution Engine::finalize(Solution out) {
  out.gpst = current_gpst_;
  out.phase = phase_;
  out.valid = out.valid && healthy_ && out.position.allFinite();
  if (out.valid && phase_ == 2 && !cfg_.gnss_motion) {
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
  current_.finalize(sats_, out.fixed);
  previous_mode_ = out.fixed ? 4 : 5;
  if (!out.fixed && phase_ == 1) {
    collection_.clear();
    fix_history_.clear();
    collecting_ = false;
  }
  ++input_epoch_;
  pruneImu();
  return out;
}
void Engine::fillCovariance(Solution &out) const {
  // Output only: marginals of the current state. A FIX keeps the covariance
  // conditioned on its integers, set where the FIX was formed.
  try {
    if (phase_ == 1) {
      if (!out.position_covariance.allFinite() && p1_) {
        const Key key = xp(input_epoch_);
        const Mat3 rot =
            p1_->calculateEstimate<gtsam::Pose3>(key).rotation().matrix();
        const gtsam::Matrix6 P = p1_->marginalCovariance(key);
        out.position_covariance =
            R_ * (rot * P.bottomRightCorner<3, 3>() * rot.transpose()) *
            R_.transpose();
      }
      return;
    }
    if (!p2_ || graph_epoch_ < 0)
      return;
    const Key pk = poseKey(graph_epoch_), vk = velKey(graph_epoch_);
    if (!p2_->getLinearizationPoint().exists(pk) ||
        !p2_->getLinearizationPoint().exists(vk))
      return;
    const Mat3 rot = p2_->calculateEstimate<gtsam::Pose3>(pk).rotation().matrix();
    const Vec3 v = p2_->calculateEstimate<Vec3>(vk);
    const auto joint = p2_->getISAM2().jointMarginalCovariance({pk, vk});
    if (!out.position_covariance.allFinite()) {
      Eigen::Matrix<double, 3, 6> J;
      J.leftCols<3>() = -rot * gtsam::skewSymmetric(lever_);
      J.rightCols<3>() = rot;
      const gtsam::Matrix6 P = joint.at(pk, pk);
      out.position_covariance = R_ * (J * P * J.transpose()) * R_.transpose();
    }
    // Antenna velocity: body velocity plus the lever-arm rotation term.
    Vec3 lever_rate = Vec3::Zero();
    if (!cfg_.gnss_motion && out.angular_velocity.allFinite())
      lever_rate = rot * out.angular_velocity.cross(lever_);
    out.velocity = R_ * (v + lever_rate);
    const Mat3 V = joint.at(vk, vk);
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
  if (phase_ == 2)
    for (auto &[sf, s] : sats_.track)
      s.generation = 0;
  const bool gnss = obs.sats.size() >= 4;
  if (!gnss) {
    std::set<SF> expired = obs.input_slips;
    for (auto &[sf, s] : sats_.track)
      if (++s.outage == 6)
        expired.insert(sf);
    for (auto sf : expired)
      sats_.get(sf).invalidate();
    if (phase_ == 1) {
      collectImu(obs.gpst);
      return finalize({});
    }
    if (!healthy_ || (cfg_.gnss_motion && !graph_gpst_))
      return finalize({});
  }
  try {
    if (phase_ == 2 && !healthy_)
      throw std::runtime_error("previous graph update failed");
    return finalize(phase_ == 1 ? phase1(obs) : phase2(obs, gnss));
  } catch (const std::exception &ex) {
    Solution out;
    out.failure = ex.what();
    if (!gnss && dynamic_cast<const ImuIntervalError *>(&ex))
      return finalize(out);
    if (phase_ == 1)
      restartPhase1();
    else if (gnss) {
      try {
        gtsam::Pose3 initial(gtsam::Rot3(),
                             R_.transpose() * (nav_position_ - base_));
        auto dd = ddprPosition(obs, initial, lever_);
        if (dd.position && std::isfinite(dd.rms) &&
            dd.rms <= cfg_.ddpr_max_res) {
          warmReset(*dd.position, initial.rotation());
          if (!cfg_.gnss_motion) collectImu(obs.gpst);
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
} // namespace gnss_fgo

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
} // namespace
std::vector<ImuSample> Engine::collectImu(double until) {
  std::vector<ImuSample> result;
  while (imu_index_ < imu_.size() && imu_[imu_index_].gpst <= until + 1e-6)
    result.push_back(imu_[imu_index_++]);
  return result;
}
void Engine::collect(const ObservationEpoch &obs, const Solution &sol) {
  auto samples = collectImu(obs.gpst);
  if (sol.fixed) {
    fix_history_.push_back({obs.gpst, sol.position});
    while (!fix_history_.empty() && obs.gpst - fix_history_.front().first > 1.5)
      fix_history_.erase(fix_history_.begin());
  }
  Vec3 velocity = Vec3::Zero();
  if (fix_history_.size() >= 2) {
    auto [t0, p0] = fix_history_.front();
    auto [t1, p1] = fix_history_.back();
    if (t1 - t0 >= .8)
      velocity = R_.transpose() * (p1 - p0) / (t1 - t0);
  }
  if (!collecting_ && velocity.head<2>().norm() > cfg_.vel_thresh && sol.fixed)
    collecting_ = true;
  if (collecting_ && sol.fixed) {
    if (!collection_.empty())
      for (auto [sf, gen] : collection_.back().arcs)
        if (sats_.get(sf).arc != gen) {
          collection_.clear();
          break;
        }
    CollectedFix fix;
    fix.time = obs.gpst;
    fix.position = sol.position;
    fix.velocity = velocity;
    fix.imu = std::move(samples);
    fix.observation = obs;
    for (auto sf : current_.visible)
      fix.arcs[sf] = sats_.get(sf).arc;
    collection_.push_back(std::move(fix));
    if (int(collection_.size()) > cfg_.n_collect)
      collection_.erase(collection_.begin());
    if (int(collection_.size()) >= cfg_.n_collect &&
        (collection_.back().position - collection_.front().position).norm() >
            2.)
      transition();
  } else if (collecting_ && !sol.fixed) {
    collecting_ = false;
    collection_.clear();
  }
}
void Engine::transition() {
  const auto saved_sats = sats_;
  const auto saved_epoch = current_;
  const auto saved_position = nav_position_;
  try {
    const auto b0 =
        stationaryBias(imu_, std::min<std::size_t>(3000, imu_index_));
    const std::size_t nstatic = std::min<std::size_t>(500, imu_index_);
    if (!nstatic)
      throw std::runtime_error("initialization needs IMU");
    Vec3 a = Vec3::Zero();
    for (std::size_t i = 0; i < nstatic; ++i)
      a += imu_[i].acc;
    a /= nstatic;
    a -= b0.accelerometer();
    double pitch = std::atan2(a.x(), std::hypot(a.y(), a.z())),
           roll = std::atan2(a.y(), a.z());
    if (std::isfinite(cfg_.init_pitch_deg))
      pitch = cfg_.init_pitch_deg * M_PI / 180.;
    const int n = collection_.size();
    std::vector<Vec3> enu;
    for (const auto &f : collection_)
      enu.push_back(R_.transpose() * (f.position - base_));
    const Vec3 total = enu.back() - enu.front();
    double previous_heading = std::atan2(total.x(), total.y());
    Graph graph;
    Values values;
    gtsam::FixedLagSmoother::KeyTimestampMap ts;
    for (int i = 0; i < n; ++i) {
      const auto &fix = collection_[i];
      const Vec3 disp = i == 0       ? Vec3(enu[1] - enu[0])
                        : i == n - 1 ? Vec3(enu[n - 1] - enu[n - 2])
                                     : Vec3(enu[i + 1] - enu[i - 1]);
      double heading = disp.head<2>().norm() > .01
                           ? std::atan2(disp.x(), disp.y())
                           : previous_heading;
      previous_heading = heading;
      const Mat3 rot = attitude(roll, pitch, heading);
      gtsam::Pose3 pose(gtsam::Rot3(rot), enu[i] - rot * lever_);
      values.insert(poseKey(i), pose);
      values.insert(velKey(i), fix.velocity);
      values.insert(biasKey(i), b0);
      const double speed = fix.velocity.head<2>().norm(),
                   rstd = cfg_.body_rot_std * M_PI / 180.;
      if (i == 0) {
        double yaw = speed > .1 ? std::hypot(rstd, .1 / speed) : .5;
        graph.addPrior<gtsam::Pose3>(poseKey(i), pose,
                                     poseNoise(.02, .02, yaw, .05, .05, .1));
      } else {
        double yaw = std::hypot(rstd, .2 / std::max(speed, .2));
        graph.addPrior<gtsam::Pose3>(poseKey(i), pose,
                                     poseNoise(.01, .01, yaw, .1, .1, .1));
      }
      graph.addPrior<gtsam::Vector3>(velKey(i), fix.velocity, noise(3, .5));
      graph.addPrior<gtsam::imuBias::ConstantBias>(biasKey(i), b0,
                                                   noise(6, .01));
      if (i > 0) {
        auto pim =
            integrateImu(cfg_, b0, fix.imu, collection_[i - 1].time, fix.time);
        graph.emplace_shared<gtsam::CombinedImuFactor>(
            poseKey(i - 1), velKey(i - 1), poseKey(i), velKey(i),
            biasKey(i - 1), biasKey(i), pim.pim);
      }
      graph.emplace_shared<gtsam::GPSFactorArm>(poseKey(i), enu[i], lever_,
                                                noise(3, .02));
      ts[poseKey(i)] = ts[velKey(i)] = ts[biasKey(i)] =
          fix.time - collection_.front().time;
    }
    for (auto &[sf, s] : sats_.track)
      s.key.reset();
    for (int i = 0; i < n; ++i)
      buildDd(graph, values, collection_[i].observation, poseKey(i), lever_);
    const double last = collection_.back().time - collection_.front().time;
    for (auto key : values.keys())
      if (!ts.count(key))
        ts[key] = last;
    double lag = cfg_.fls_lag;
    if (lag + 1e-6 < last + dt_)
      lag = last + 2 * dt_;
    auto smoother = makeSmoother(lag, cfg_);
    smoother->update(graph, values, ts);
    auto est = smoother->calculateEstimate();
    auto p = est.at<gtsam::Pose3>(poseKey(n - 1));
    auto v = est.at<gtsam::Vector3>(velKey(n - 1));
    auto b = est.at<gtsam::imuBias::ConstantBias>(biasKey(n - 1));
    if (!p.matrix().allFinite() || !v.allFinite() || !b.vector().allFinite())
      throw std::runtime_error("invalid initialization state");
    nav_position_ = antenna(p);
    p2_ = std::move(smoother);
    graph_time_ = last;
    graph_epoch_ = n - 1;
    graph_gpst_ = collection_.back().time;
    bias_ = initial_bias_ = b0;
    actual_lag_ = lag;
    phase_ = 2;
    healthy_ = true;
    collecting_ = false;
    collection_.clear();
    fix_history_.clear();
  } catch (...) {
    sats_ = saved_sats;
    current_ = saved_epoch;
    nav_position_ = saved_position;
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
  if (!graph_gpst_ && !cfg_.gnss_motion)
    throw std::runtime_error("committed graph time missing");
  const double next = graph_gpst_ ? graph_time_ + current_gpst_ - *graph_gpst_ : 0.;
  gtsam::FixedLagSmoother::KeyTimestampMap ts;
  gtsam::FactorIndices slots;
  ts[poseKey(k)] = ts[velKey(k)] = next;
  if (!cfg_.gnss_motion) ts[biasKey(k)] = next;
  if (k > 0) {
    ts[poseKey(k - 1)] = ts[velKey(k - 1)] = next;
    if (!cfg_.gnss_motion) ts[biasKey(k - 1)] = next;
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
    if (cfg_.zupt_g_dev_thr > 0)
      stationary =
          stationary && std::abs((acc - initial_bias_.accelerometer()).norm() -
                                 9.81) <= cfg_.zupt_g_dev_thr;
  }
  if (!stationary) {
    zupt_anchor_.reset();
    return;
  }
  if (cfg_.zupt_sigma_zero_velocity > 0)
    graph.addPrior<gtsam::Vector3>(velKey(k), Vec3::Zero(),
                                   noise(3, cfg_.zupt_sigma_zero_velocity));
  if (cfg_.zupt_sigma_zero_rotation > 0 && k > 0)
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        poseKey(k - 1), poseKey(k), gtsam::Pose3(),
        splitNoise(cfg_.zupt_sigma_zero_rotation, 1e3));
  if (cfg_.zupt_anchor_sigma_translation > 0 &&
      cfg_.zupt_anchor_sigma_rotation > 0) {
    if (!zupt_anchor_)
      zupt_anchor_ = previous;
    else
      graph.addPrior<gtsam::Pose3>(
          poseKey(k), *zupt_anchor_,
          splitNoise(cfg_.zupt_anchor_sigma_rotation,
                     cfg_.zupt_anchor_sigma_translation));
  }
}
void Engine::addDoppler(Graph &graph, int k, const ObservationEpoch &obs,
                        const gtsam::NavState &pred, const Vec3 &gyro, int nv) {
  if (cfg_.doppler_sd_sigma <= 0 ||
      (cfg_.doppler_require_dd && nv < cfg_.min_dd_for_solve))
    return;
  struct Row {
    const SatelliteObservation *sat;
    double d, lam, snr;
  };
  std::vector<Row> rows;
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
    return;
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
      b[i] = -row.lam * row.d - e.dot(row.sat->sat_velocity - vpred);
    }
    Eigen::VectorXd x =
        A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    Eigen::VectorXd residual = A * x - b;
    std::vector<double> r(residual.data(), residual.data() + residual.size());
    const double mid = median(r);
    for (auto &val : r)
      val = std::abs(val - mid);
    scale = 1.4826 * median(r);
    if (cfg_.doppler_fde_k > 0 && scale > 0) {
      std::vector<Row> kept;
      for (std::size_t i = 0; i < rows.size(); ++i)
        if (std::abs(residual[i]) <= cfg_.doppler_fde_k * scale)
          kept.push_back(rows[i]);
      if (kept.size() >= 4)
        rows = std::move(kept);
    }
  }
  std::vector<double> sigmas;
  for (auto &row : rows) {
    double sigma = std::max(cfg_.doppler_sd_sigma,
                            cfg_.doppler_adaptive_sigma ? scale : 0.);
    sigma /= std::max(std::sin(row.sat->elevation), .1);
    if (cfg_.doppler_snr_weight && row.snr > 0)
      sigma *= std::clamp(std::pow(10., (cfg_.snr_ref_dbhz - row.snr) / 20.),
                          1., 10.);
    sigmas.push_back(sigma);
  }
  auto ref_index =
      std::min_element(sigmas.begin(), sigmas.end()) - sigmas.begin();
  const auto &ref = rows[ref_index];
  for (std::size_t i = 0; i < rows.size(); ++i) {
    if (i == std::size_t(ref_index))
      continue;
    const auto &row = rows[i];
    gtsam::SharedNoiseModel model =
        noise(1, std::hypot(sigmas[i], sigmas[ref_index]));
    if (cfg_.doppler_huber > 0)
      model = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(cfg_.doppler_huber),
          model);
    graph.emplace_shared<SdDopplerFactor>(
        poseKey(k), velKey(k), row.d, ref.d, row.lam, ref.lam,
        row.sat->rover_sat, row.sat->sat_velocity, ref.sat->rover_sat,
        ref.sat->sat_velocity, receiver, lever_, R_, gyro - bias_.gyroscope(),
        model);
  }
}
std::pair<double, std::map<int, double>>
Engine::residuals(const Graph &graph, const Values &values) const {
  double sum = 0;
  int n = 0;
  std::map<int, double> per_sat;
  for (auto fac : graph) {
    auto f = std::dynamic_pointer_cast<DdFactor>(fac);
    if (!f || f->carrier)
      continue;
    double r = std::sqrt(2. * std::max(0., f->error(values))) * cfg_.sigma_pr *
               std::sqrt(2.);
    sum += r * r;
    ++n;
    for (int sat : {f->ref_sat, f->target_sat})
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
  const auto &graph = p2_->getFactors();
  for (std::size_t i = 0; i < graph.size(); ++i)
    if (graph[i])
      for (auto key : graph[i]->keys())
        if (keys.count(key)) {
          remove.push_back(i);
          break;
        }
  try {
    if (!remove.empty()) {
      updateProbe("reset_ambiguities", false);
      p2_->update(Graph{}, Values{}, gtsam::FixedLagSmoother::KeyTimestampMap{},
                  remove);
      updateProbe("reset_ambiguities", true);
    }
  } catch (...) {
    healthy_ = false;
    throw;
  }
  for (auto &[sf, s] : sats_.track)
    s.invalidate();
  current_.accepted.clear();
  cp_hold_ = cfg_.recov_cp_hold;
}
void Engine::warmReset(const Vec3 &position, const gtsam::Rot3 &rotation) {
  if (cfg_.gnss_motion) {
    auto replacement = makeSmoother(actual_lag_, cfg_);
    Graph graph; Values values;
    gtsam::Pose3 pose(gtsam::Rot3(), R_.transpose() * (position - base_));
    values.insert(poseKey(0), pose);
    values.insert(velKey(0), Vec3::Zero().eval());
    graph.addPrior<gtsam::Pose3>(poseKey(0), pose, poseNoise(.01,.01,.01,100,100,100));
    graph.addPrior<Vec3>(velKey(0), Vec3::Zero(), noise(3,100));
    updateProbe("warm_reset", false);
    replacement->update(graph, values, {{poseKey(0),0.},{velKey(0),0.}});
    updateProbe("warm_reset", true);
    const auto estimate = replacement->calculateEstimate();
    if (!estimate.at<gtsam::Pose3>(poseKey(0)).matrix().allFinite() || !estimate.at<Vec3>(velKey(0)).allFinite())
      throw std::runtime_error("invalid motion reset");
    p2_ = std::move(replacement); graph_epoch_ = 0; graph_time_ = 0.;
    graph_gpst_ = current_gpst_; healthy_ = true;
    for (auto &[sf,s] : sats_.track) s.invalidate();
    current_.accepted.clear(); current_.visible.clear();
    zupt_anchor_.reset(); per_sat_res_.clear();
    cp_hold_ = cfg_.recov_cp_hold; pim_break_ = false;
    nav_position_ = position;
    return;
  }
  gtsam::Pose3 pose(rotation, R_.transpose() * (position - base_));
  auto replacement = makeSmoother(actual_lag_, cfg_);
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
std::optional<Solution> Engine::sanity(const Graph &graph,
                                       const gtsam::Pose3 &pose,
                                       const gtsam::NavState &pred, int k) {
  if (!cfg_.ddpr_sanity_enable)
    return {};
  if (main_res_ <= cfg_.main_ddpr_res_thresh) {
    bad_count_ = 0;
    return {};
  }
  std::vector<double> r;
  for (auto [s, v] : ar_per_sat_res_)
    r.push_back(v);
  std::sort(r.begin(), r.end());
  if (cfg_.sanity_max_median_ratio > 0 &&
      int(r.size()) >= cfg_.sanity_max_median_min_sats &&
      r[r.size() / 2] > 1e-3 &&
      r.back() / r[r.size() / 2] > cfg_.sanity_max_median_ratio)
    return {};
  Values p;
  p.insert(poseKey(k), pred.pose());
  double pred_res = residuals(graph, p).first;
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
  if (cfg_.sanity_break_pim)
    pim_break_ = true;
  const bool replace = cfg_.sanity_pose_replace_thresh > 0 &&
                       pred_res <= cfg_.sanity_pose_replace_thresh &&
                       (pose.translation() - pred.pose().translation()).norm() >
                           cfg_.sanity_pose_replace_thresh;
  Solution sol;
  sol.position = antenna(replace ? pred.pose() : pose);
  sol.valid = true;
  sol.kind = replace ? UpdateKind::DeadReckoning : UpdateKind::Gnss;
  nav_position_ = antenna(pose);
  return sol;
}
Solution Engine::phase2(const ObservationEpoch &obs, bool gnss) {
  const int k = graph_epoch_ + 1;
  Values estimate;
  gtsam::Pose3 previous;
  Vec3 velocity = Vec3::Zero();
  gtsam::imuBias::ConstantBias previous_bias;
  gtsam::NavState pred;
  Graph graph; Values values;
  std::vector<ImuSample> samples;
  IntegratedImu integrated(cfg_, bias_);
  if (cfg_.gnss_motion) {
    if (!graph_gpst_) {
      gtsam::Pose3 seed(gtsam::Rot3(), R_.transpose()*(nav_position_-base_));
      const auto dd = ddprPosition(obs, seed, Vec3::Zero());
      if (!dd.position || !dd.position->allFinite()) return {};
      seed = gtsam::Pose3(gtsam::Rot3(), R_.transpose()*(*dd.position-base_));
      pred = gtsam::NavState(seed, Vec3::Zero());
      graph.addPrior<gtsam::Pose3>(poseKey(k), seed, poseNoise(.01,.01,.01,100,100,100));
      graph.addPrior<Vec3>(velKey(k), Vec3::Zero(), noise(3,100));
    } else {
      estimate = p2_->calculateEstimate();
      previous = estimate.at<gtsam::Pose3>(poseKey(k-1));
      velocity = estimate.at<Vec3>(velKey(k-1));
      const double elapsed = obs.gpst - *graph_gpst_;
      pred = gtsam::NavState(gtsam::Pose3(gtsam::Rot3(),previous.translation()+elapsed*velocity),velocity);
      graph.emplace_shared<MotionFactor>(poseKey(k-1),velKey(k-1),poseKey(k),velKey(k),elapsed,cfg_.motion_acceleration_psd);
      graph.emplace_shared<gtsam::PoseRotationPrior<gtsam::Pose3>>(poseKey(k),gtsam::Rot3(),noise(3,.01));
    }
    values.insert(poseKey(k),pred.pose());
    values.insert(velKey(k),Vec3(pred.velocity()));
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
  std::vector<Key> keep;
  std::map<SF, std::pair<Key, double>> old;
  if (gnss) {
    slips(obs);
    current_.cp_suppressed = cp_hold_ > 0;
    if (current_.cp_suppressed)
      --cp_hold_;
    for (auto [sf, key] : sats_.keys())
      if (current_.cp_suppressed)
        ++sats_.get(sf).generation;
      else if (estimate.exists(key))
        old[sf] = {key, estimate.at<double>(key)};
    for (auto &[sf, s] : sats_.track)
      s.key.reset();
    nv = buildDd(graph, values, obs, poseKey(k), lever_, k, old,
                 current_.cp_suppressed);
    if (!current_.cp_suppressed && cfg_.betweenn_enable)
      for (auto [sf, key] : sats_.keys())
        if (old.count(sf))
          graph.emplace_shared<gtsam::BetweenFactor<double>>(
              old[sf].first, key, 0.,
              noise(1, previous_mode_ == 5 ? cfg_.sigma_n_between_flt
                                           : cfg_.sigma_n_between));
    if (nv < cfg_.min_dd_for_solve && !cfg_.gnss_motion) {
      graph.addPrior<gtsam::Pose3>(poseKey(k), pred.pose(),
                                   noise(6, cfg_.propagate_pose_sigma));
      graph.addPrior<Vec3>(velKey(k), pred.velocity(),
                           noise(3, cfg_.propagate_vel_sigma));
      graph.addPrior<gtsam::imuBias::ConstantBias>(
          biasKey(k), previous_bias, noise(6, cfg_.propagate_bias_sigma));
      if (!current_.cp_suppressed)
        for (auto [sf, key] : sats_.keys())
          if (old.count(sf))
            graph.addPrior<double>(key, old[sf].second,
                                   noise(1, cfg_.propagate_amb_sigma));
    }
    addDoppler(graph, k, obs, pred, integrated.mean_gyro, nv);
  } else if (!cfg_.gnss_motion)
    graph.addPrior<gtsam::imuBias::ConstantBias>(biasKey(k), bias_,
                                                 splitNoise(1e-4, 3e-6));
  if (!cfg_.gnss_motion) addPseudo(graph, k, previous, velocity, samples, integrated.mean_gyro);
  for (auto [sf, key] : sats_.keys())
    keep.push_back(key);
  if (gnss)
    for (auto [sf, kv] : old)
      if (sats_.get(sf).key)
        keep.push_back(kv.first);
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
    const auto dd = ddprPosition(obs, pred.pose(), lever_);
    if (dd.position && std::isfinite(dd.rms) && dd.rms <= cfg_.ddpr_max_res) {
      try {
        warmReset(*dd.position, pred.pose().rotation());
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
  if (gnss) {
    auto res = residuals(graph, estimate);
    main_res_ = res.first;
    ar_per_sat_res_ = res.second;
    residual_sat_order_.clear();
    std::set<int> seen;
    for (auto fac : graph) {
      auto f = std::dynamic_pointer_cast<DdFactor>(fac);
      if (!f || f->carrier)
        continue;
      for (int s : {f->ref_sat, f->target_sat})
        if (ar_per_sat_res_.count(s) && seen.insert(s).second)
          residual_sat_order_.push_back(s);
    }
    estimate = fde(*p2_, graph, slots, estimate, poseKey(k), nv);
  }
  auto pose = estimate.at<gtsam::Pose3>(poseKey(k));
  auto vel = estimate.at<Vec3>(velKey(k));
  if (!cfg_.gnss_motion) bias_ = estimate.at<gtsam::imuBias::ConstantBias>(biasKey(k));
  if (!pose.matrix().allFinite() || !vel.allFinite() ||
      !bias_.vector().allFinite())
    throw std::runtime_error("nonfinite TC posterior");
  nav_position_ = antenna(pose);
  ArDecision ar;
  if (gnss && k >= (cfg_.gnss_motion ? 5 : cfg_.n_collect + 3) && !current_.cp_suppressed && !cp_hold_) {
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
    if (cfg_.ar_gdop_max <= 0 || gdop <= cfg_.ar_gdop_max) {
      ar = runAr(*p2_, estimate, poseKey(k), obs);
      if (ar.nb)
        starvation_ = 0;
      else if (ar_outcome_ == ArOutcome::Zero && cfg_.ar_starve_reset > 0) {
        ++starvation_;
        if (starvation_ >= cfg_.ar_starve_reset && !cp_hold_ &&
            main_res_ <= cfg_.ar_starve_max_res) {
          resetAmbiguities();
          starvation_ = 0;
        }
      }
    }
  }
  if (gnss && !ar.nb) {
    auto recovery = sanity(graph, pose, pred, k);
    if (recovery)
      return *recovery;
  }
  if (ar.nb)
    hold(*p2_, ar);
  else if (gnss) {
    const double threshold = std::max(2., .5 * cfg_.ar_context_worst_sat_max);
    int worst = 0;
    double worst_res = 0;
    for (auto s : residual_sat_order_)
      if (ar_per_sat_res_[s] > worst_res) {
        worst = s;
        worst_res = ar_per_sat_res_[s];
      }
    std::optional<std::tuple<double, double, int, int>> best;
    for (auto sf : sats_.insertion_order)
      if (sats_.get(sf).held) {
        double r = ar_per_sat_res_[sf.first], score = r >= threshold ? r : 0.;
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
  return sol;
}
} // namespace gnss_fgo
