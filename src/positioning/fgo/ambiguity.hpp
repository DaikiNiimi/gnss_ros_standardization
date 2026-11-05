#pragma once
// Integer ambiguity resolution: LDL reduction, integer least-squares search and
// partial-ambiguity policy. Bounds, tie rounding and partial-search return values
// deliberately match the frozen Python baseline rather than RTKLIB lambda().
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace gnss_fgo::ar {
struct Reduced {
  bool valid = false;
  int iterations = 0;
  Eigen::MatrixXd L, Z;
  Eigen::VectorXd d;
};
inline Reduced boundedReduction(const Eigen::MatrixXd &Q) {
  Reduced r;
  const int n = Q.rows();
  if (n < 1 || Q.cols() != n || !Q.allFinite())
    return r;
  Eigen::MatrixXd work = Q;
  r.L = Eigen::MatrixXd::Zero(n, n);
  r.d.resize(n);
  r.Z = Eigen::MatrixXd::Identity(n, n);
  for (int i = n - 1; i >= 0; --i) {
    r.d(i) = work(i, i);
    if (!(r.d(i) >= 1e-10))
      return r;
    const double root = std::sqrt(r.d(i));
    for (int j = 0; j <= i; ++j)
      r.L(i, j) = work(i, j) / root;
    for (int j = 0; j < i; ++j)
      for (int k = 0; k <= j; ++k)
        work(j, k) -= r.L(i, k) * r.L(i, j);
    const double diag = r.L(i, i);
    for (int j = 0; j <= i; ++j)
      r.L(i, j) /= diag;
  }
  int j = n - 2, k = n - 2;
  while (j >= 0 && r.iterations < 10000) {
    ++r.iterations;
    if (j <= k)
      for (int i = j + 1; i < n; ++i) {
        const double mu = std::nearbyint(r.L(i, j));
        if (mu != 0.) {
          for (int row = i; row < n; ++row)
            r.L(row, j) -= mu * r.L(row, i);
          for (int row = 0; row < n; ++row)
            r.Z(row, j) -= mu * r.Z(row, i);
        }
      }
    const double delta = r.d(j) + r.L(j + 1, j) * r.L(j + 1, j) * r.d(j + 1);
    if (delta + 1e-6 < r.d(j + 1)) {
      const double eta = r.d(j) / delta,
                   lam = r.d(j + 1) * r.L(j + 1, j) / delta;
      r.d(j) = eta * r.d(j + 1);
      r.d(j + 1) = delta;
      const double previous = r.L(j + 1, j);
      for (int col = 0; col < j; ++col) {
        const double a = r.L(j, col), b = r.L(j + 1, col);
        r.L(j, col) = -previous * a + b;
        r.L(j + 1, col) = eta * a + lam * b;
      }
      r.L(j + 1, j) = lam;
      for (int row = j + 2; row < n; ++row)
        std::swap(r.L(row, j), r.L(row, j + 1));
      for (int row = 0; row < n; ++row)
        std::swap(r.Z(row, j), r.Z(row, j + 1));
      k = j;
      j = n - 2;
    } else
      --j;
  }
  r.valid = r.L.allFinite() && r.d.allFinite() && r.Z.allFinite() &&
            (r.d.array() > 0).all();
  return r;
}
} // namespace gnss_fgo::ar

// deliberately match the frozen Python baseline rather than RTKLIB lambda().
namespace gnss_fgo::ar {
struct IntegerSearch {
  Eigen::MatrixXd candidates;
  Eigen::Vector2d cost = Eigen::Vector2d::Constant(1e18);
};
inline IntegerSearch estimILS(const Eigen::MatrixXd &L,
                              const Eigen::VectorXd &d,
                              const Eigen::VectorXd &a) {
  const int n = d.size();
  IntegerSearch out;
  out.candidates = Eigen::MatrixXd::Zero(n, 2);
  Eigen::VectorXd acond = Eigen::VectorXd::Zero(n),
                  left = Eigen::VectorXd::Zero(n),
                  dist = Eigen::VectorXd::Zero(n);
  Eigen::VectorXi z = Eigen::VectorXi::Zero(n), step = Eigen::VectorXi::Zero(n),
                  path = Eigen::VectorXi::Constant(n, n - 1);
  Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n, n);
  auto sign = [](double v) { return v > 0 ? 1 : v < 0 ? -1 : 0; };
  acond[n - 1] = a[n - 1];
  z[n - 1] = int(std::nearbyint(acond[n - 1]));
  left[n - 1] = acond[n - 1] - z[n - 1];
  step[n - 1] = sign(left[n - 1]);
  if (!step[n - 1])
    step[n - 1] = 1;
  int count = -1, imax = 1, k = n - 1, loops = 0;
  double chi = 1e18;
  bool end = false, aborted = false;
  while (!end) {
    double nd = dist[k] + left[k] * left[k] / d[k];
    while (nd < chi) {
      if (++loops > 10000) {
        aborted = end = true;
        break;
      }
      if (k != 0) {
        --k;
        dist[k] = nd;
        for (int j = path[k]; j > k; --j)
          S(j - 1, k) = S(j, k) - left[j] * L(j, k);
        acond[k] = a[k] + S(k, k);
        z[k] = int(std::nearbyint(acond[k]));
        left[k] = acond[k] - z[k];
        step[k] = sign(left[k]);
        if (!step[k])
          step[k] = 1;
      } else {
        if (count < 0) {
          ++count;
          out.candidates.col(count) = z.cast<double>();
          out.cost[count] = nd;
        } else {
          out.candidates.col(imax) = z.cast<double>();
          out.cost[imax] = nd;
          imax = out.cost[0] >= out.cost[1] ? 0 : 1;
          chi = out.cost[imax];
        }
        k = 0;
        z[k] += step[k];
        left[k] = acond[k] - z[k];
        step[k] = -step[k] - sign(step[k]);
      }
      nd = dist[k] + left[k] * left[k] / d[k];
    }
    if (aborted)
      break;
    int level = k;
    while (nd >= chi) {
      if (++loops > 10000) {
        aborted = end = true;
        break;
      }
      if (k == n - 1) {
        end = true;
        break;
      }
      ++k;
      z[k] += step[k];
      left[k] = acond[k] - z[k];
      step[k] = -step[k] - sign(step[k]);
      nd = dist[k] + left[k] * left[k] / d[k];
    }
    if (aborted)
      break;
    for (int i = level; i < k; ++i)
      path[i] = k;
    for (int j = level - 1; j >= 0; --j) {
      if (path[j] < k)
        path[j] = k;
      else
        break;
    }
  }
  if (out.cost[0] > out.cost[1]) {
    std::swap(out.cost[0], out.cost[1]);
    out.candidates.col(0).swap(out.candidates.col(1));
  }
  return out;
}
} // namespace gnss_fgo::ar

namespace gnss_fgo::ar {
struct Result {
  bool accepted = false;
  int dropped = 0, kept = 0, error = 0;
  double s0 = 0, s1 = 0, threshold = 3;
  Eigen::VectorXd conditioned, integers;
  Eigen::MatrixXd A; // fixed integer combinations: A * original_DD = integers
  std::vector<unsigned char>
      exact_rows; // only these ORIGINAL DD rows may be held
};
inline double ratioThreshold(int n, bool adaptive) {
  if (!adaptive)
    return 3.;
  const double coeff[3][5] = {
      {-0.194058448, -7.79023476, 124.23112, -403.12605, 350.413202},
      {0.642237302, -8.39813962, 29.2107285, -23.7577308, -1.14307128},
      {-0.022260039, 0.323169103, -1.39837429, 2.19282996, -0.0534583971}};
  double c[3];
  for (int i = 0; i < 3; ++i) {
    c[i] = coeff[i][0];
    for (int j = 1; j < 5; ++j)
      c[i] = c[i] * 3. + coeff[i][j];
  }
  double t = c[0];
  for (int i = 1; i < 3; ++i)
    t = t / (std::min(n, 50) + 1) + c[i];
  return std::clamp(t, 1.5, 3.);
}
inline Result resolve(const Eigen::VectorXd &a, const Eigen::MatrixXd &input,
                      bool adaptive, bool allow_partial, int min_full = 4) {
  Result out;
  const int n = a.size();
  if (n < std::max(1, min_full) || input.rows() != n || input.cols() != n ||
      !a.allFinite() || !input.allFinite())
    return out;
  if (!input.isApprox(input.transpose(), 1e-10)) {
    out.error = -10;
    return out;
  }
  const Eigen::MatrixXd Q = input;
  Eigen::LLT<Eigen::MatrixXd> initial(Q);
  if (initial.info() != Eigen::Success) {
    out.error = -11;
    return out;
  }
  const auto reduced = boundedReduction(Q);
  if (!reduced.valid) {
    out.error = -13;
    return out;
  }
  const Eigen::MatrixXd Z = reduced.Z, Zt = Z.transpose();
  const Eigen::MatrixXd inverse = Zt.partialPivLu().inverse().unaryExpr(
      [](double x) { return std::nearbyint(x); });
  const Eigen::VectorXd z = Zt * a, d = reduced.d;
  const auto searched = estimILS(reduced.L, d, z);
  const Eigen::MatrixXd F = inverse * searched.candidates;
  const auto sq = searched.cost;
  out.s0 = sq[0];
  out.s1 = sq[1];
  out.threshold = ratioThreshold(n, adaptive);
  const bool exact = sq[0] <= 1e-12 * std::max(sq[1], 1.);
  if (exact || (sq[0] > 0 && sq[1] / sq[0] >= out.threshold)) {
    out.accepted = true;
    out.kept = n;
    out.conditioned = F.col(0);
    out.integers = F.col(0);
    out.A = Eigen::MatrixXd::Identity(n, n);
    out.exact_rows.assign(n, 1);
    return out;
  }
  constexpr int min_partial = 20, max_trials = 8;
  constexpr double p0 = .995;
  if (!allow_partial || n <= min_partial)
    return out;
  if (!inverse.allFinite() ||
      inverse.cwiseAbs().maxCoeff() > 4503599627370496. ||
      !(Zt * inverse).isApprox(Eigen::MatrixXd::Identity(n, n), 1e-9)) {
    out.error = -12;
    return out;
  }
  Eigen::MatrixXd Qz = reduced.L.transpose() * d.asDiagonal() * reduced.L;
  auto success = [&](int k) {
    double logp = 0.;
    for (int i = k; i < n; ++i) {
      const double v = std::erf(.5 / std::sqrt(2. * d(i)));
      if (!(v > 0))
        return 0.;
      logp += std::log(v);
    }
    return std::exp(logp);
  };
  const int kmax = n - min_partial;
  int k = 1;
  while (k < kmax && success(k) < p0)
    ++k;
  for (int attempt = 0; attempt < max_trials && k <= kmax; ++attempt) {
    const int kept = n - k;
    const Eigen::MatrixXd Qt = Qz.bottomRightCorner(kept, kept);
    const Eigen::VectorXd zt = z.tail(kept);
    const auto search =
        estimILS(reduced.L.bottomRightCorner(kept, kept), d.tail(kept), zt);
    const auto candidates = search.candidates;
    const auto cost = search.cost;
    const double threshold = ratioThreshold(kept, adaptive);
    const bool exact_subset =
        cost[0] == 0. && std::isfinite(cost[1]) && cost[1] > 0.;
    if (exact_subset || (cost[0] > 0 && cost[1] / cost[0] >= threshold)) {
      Eigen::LDLT<Eigen::MatrixXd> solve(Qt);
      if (solve.info() != Eigen::Success || solve.vectorD().minCoeff() <= 0) {
        out.error = -14;
        return out;
      }
      Eigen::VectorXd conditioned_z(n);
      conditioned_z.head(k) =
          z.head(k) -
          Qz.topRightCorner(k, kept) * solve.solve(zt - candidates.col(0));
      conditioned_z.tail(kept) = candidates.col(0);
      out.conditioned = inverse * conditioned_z;
      out.A = Z.rightCols(kept).transpose();
      out.integers = candidates.col(0);
      out.exact_rows.resize(n);
      for (int i = 0; i < n; ++i)
        out.exact_rows[i] = (inverse.row(i).head(k).array() == 0.).all();
      out.accepted = true;
      out.kept = kept;
      out.dropped = k;
      out.s0 = cost[0];
      out.s1 = cost[1];
      out.threshold = threshold;
      return out;
    }
    k += std::max(1, (kmax - k) / 4);
  }
  return out;
}
} // namespace gnss_fgo::ar
