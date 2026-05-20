#include "errState_extended_kalman.hpp"

#include <numeric>
#include <stdexcept>

namespace tools
{

ErrorStateExtendedKalmanFilter::ErrorStateExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0, InjectFunction inject,
  ResetJacobianFunction reset_jacobian)
: x(x0),
  P(P0),
  dx(Eigen::VectorXd::Zero(P0.rows())),
  last_correction(Eigen::VectorXd::Zero(P0.rows())),
  I(Eigen::MatrixXd::Identity(P0.rows(), P0.rows())),
  inject_(inject),
  reset_jacobian_(reset_jacobian)
{
  if (x0.size() == 0) {
    throw std::invalid_argument("ErrorStateExtendedKalmanFilter: x0 must not be empty");
  }
  if (P0.rows() == 0 || P0.rows() != P0.cols()) {
    throw std::invalid_argument(
      "ErrorStateExtendedKalmanFilter: P0 must be a non-empty square matrix");
  }

  data["nis"] = 0.0;
  data["nis_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
  data["correction_norm"] = 0.0;
  data["residual_norm"] = 0.0;
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
}

Eigen::VectorXd ErrorStateExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [](const Eigen::VectorXd & x) { return x; });
}

Eigen::VectorXd ErrorStateExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q, StateFunction f)
{
  checkInitialized();
  const int error_dim = errorDim();
  if (F.rows() != error_dim || F.cols() != error_dim) {
    throw std::invalid_argument(
      "ErrorStateExtendedKalmanFilter::predict: F size must match error-state dimension");
  }
  if (Q.rows() != error_dim || Q.cols() != error_dim) {
    throw std::invalid_argument(
      "ErrorStateExtendedKalmanFilter::predict: Q size must match error-state dimension");
  }

  x = f(x);
  P = F * P * F.transpose() + Q;
  P = 0.5 * (P + P.transpose());
  resetError();
  return x;
}

Eigen::VectorXd ErrorStateExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  SubtractFunction z_subtract)
{
  return update(
    z, H, R,
    [&](const Eigen::VectorXd & x) {
      if (H.cols() != x.size()) {
        throw std::invalid_argument(
          "ErrorStateExtendedKalmanFilter::update: linear update requires nominal-state and "
          "error-state dimensions to be equal; pass h(x) for a true error-state measurement "
          "model");
      }
      return H * x;
    },
    z_subtract);
}

Eigen::VectorXd ErrorStateExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  StateFunction h, SubtractFunction z_subtract)
{
  checkInitialized();
  const int error_dim = errorDim();
  if (H.cols() != error_dim) {
    throw std::invalid_argument(
      "ErrorStateExtendedKalmanFilter::update: H columns must match error-state dimension");
  }
  if (H.rows() != z.size()) {
    throw std::invalid_argument(
      "ErrorStateExtendedKalmanFilter::update: H rows must match measurement dimension");
  }
  if (R.rows() != z.size() || R.cols() != z.size()) {
    throw std::invalid_argument(
      "ErrorStateExtendedKalmanFilter::update: R size must match measurement dimension");
  }

  const Eigen::VectorXd residual = z_subtract(z, h(x));
  const Eigen::MatrixXd S = H * P * H.transpose() + R;
  const Eigen::MatrixXd S_inv =
    S.ldlt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
  const Eigen::MatrixXd K = P * H.transpose() * S_inv;
  const Eigen::VectorXd error = K * residual;
  const Eigen::MatrixXd I_KH = I - K * H;

  P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
  P = 0.5 * (P + P.transpose());

  x = inject_(x, error);
  dx = error;
  last_correction = error;

  if (reset_jacobian_) {
    const Eigen::MatrixXd G = reset_jacobian_(error);
    if (G.rows() != error_dim || G.cols() != error_dim) {
      throw std::invalid_argument(
        "ErrorStateExtendedKalmanFilter::update: reset jacobian size must match error-state "
        "dimension");
    }
    P = G * P * G.transpose();
    P = 0.5 * (P + P.transpose());
  }

  updateDiagnostics(residual, S, error);
  resetError();
  return x;
}

void ErrorStateExtendedKalmanFilter::resetError()
{
  if (P.rows() > 0) {
    dx = Eigen::VectorXd::Zero(P.rows());
  }
}

int ErrorStateExtendedKalmanFilter::errorDim() const { return static_cast<int>(P.rows()); }

void ErrorStateExtendedKalmanFilter::checkInitialized() const
{
  if (x.size() == 0 || P.rows() == 0 || P.cols() == 0 || I.rows() == 0) {
    throw std::runtime_error("ErrorStateExtendedKalmanFilter is not initialized");
  }
}

void ErrorStateExtendedKalmanFilter::updateDiagnostics(
  const Eigen::VectorXd & residual, const Eigen::MatrixXd & S, const Eigen::VectorXd & error)
{
  const double nis = residual.transpose() * S.ldlt().solve(residual);
  const bool nis_failed = nis > nis_threshold;
  if (nis_failed) {
    nis_count_++;
  }
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis_failed ? 1 : 0);
  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  const int recent_failures =
    std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  const double recent_rate =
    recent_nis_failures.empty()
      ? 0.0
      : static_cast<double>(recent_failures) / static_cast<double>(recent_nis_failures.size());

  data["nis"] = nis;
  data["nis_fail"] = nis_failed ? 1.0 : 0.0;
  data["nis_fail_rate"] =
    total_count_ == 0 ? 0.0 : static_cast<double>(nis_count_) / static_cast<double>(total_count_);
  data["recent_nis_failures"] = recent_rate;
  data["correction_norm"] = error.norm();
  data["residual_norm"] = residual.norm();

  data["residual_yaw"] = residual.size() > 0 ? residual[0] : 0.0;
  data["residual_pitch"] = residual.size() > 1 ? residual[1] : 0.0;
  data["residual_distance"] = residual.size() > 2 ? residual[2] : 0.0;
  data["residual_angle"] = residual.size() > 3 ? residual[3] : 0.0;

  for (int i = 0; i < residual.size(); ++i) {
    data["residual_" + std::to_string(i)] = residual[i];
  }
}

}  // namespace tools
