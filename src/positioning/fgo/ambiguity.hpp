#pragma once
// Integer ambiguity resolution with RTKLIB's LAMBDA and ratio test.
#include "gnss_ros_standardization/rtklib_compat.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <algorithm>

namespace gnss_fgo::ar {
struct Result {
  bool accepted = false;
  int kept = 0, error = 0;
  double s0 = 0, s1 = 0;
  Eigen::VectorXd integers;
};
inline Result resolve(const Eigen::VectorXd &a, const Eigen::MatrixXd &input,
                      int min_full = 4, double ratio = 3.) {
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
  Eigen::MatrixXd F(n, 2);
  Eigen::Vector2d sq;
  if (lambda(n, 2, a.data(), Q.data(), F.data(), sq.data())) {
    out.error = -13;
    return out;
  }
  out.s0 = sq[0];
  out.s1 = sq[1];
  if (sq[0] <= 0 || sq[1] / sq[0] >= ratio) {
    out.accepted = true;
    out.kept = n;
    out.integers = F.col(0);
  }
  return out;
}
} // namespace gnss_fgo::ar
