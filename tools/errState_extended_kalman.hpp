#ifndef TOOLS__ERRSTATE_EXTENDED_KALMAN_HPP
#define TOOLS__ERRSTATE_EXTENDED_KALMAN_HPP

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <string>

namespace tools
{

class ErrorStateExtendedKalmanFilter
{
public:
  using StateFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
  using InjectFunction =
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)>;
  using SubtractFunction =
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)>;
  using ResetJacobianFunction = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;

  Eigen::VectorXd x;  // nominal state
  Eigen::MatrixXd P;  // error-state covariance
  Eigen::VectorXd dx;  // current error state, reset to zero after injection
  Eigen::VectorXd last_correction;

  ErrorStateExtendedKalmanFilter() = default;

  ErrorStateExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    InjectFunction inject = [](const Eigen::VectorXd & nominal,
                                const Eigen::VectorXd & error) {
      Eigen::VectorXd corrected = nominal;
      corrected += error;
      return corrected;
    },
    ResetJacobianFunction reset_jacobian = nullptr);

  Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q);

  Eigen::VectorXd predict(
    const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q, StateFunction f);

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    SubtractFunction z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
      return a - b;
    });

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    StateFunction h,
    SubtractFunction z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
      return a - b;
    });

  void resetError();
  int errorDim() const;

  std::map<std::string, double> data;
  std::deque<int> recent_nis_failures{0};
  size_t window_size = 100;
  double last_nis = 0.0;
  double nis_threshold = std::numeric_limits<double>::infinity();

private:
  Eigen::MatrixXd I;
  InjectFunction inject_;
  ResetJacobianFunction reset_jacobian_;

  int nis_count_ = 0;
  int total_count_ = 0;

  void checkInitialized() const;
  void updateDiagnostics(
    const Eigen::VectorXd & residual, const Eigen::MatrixXd & S,
    const Eigen::VectorXd & error);
};

using ErrStateExtendedKalmanFilter = ErrorStateExtendedKalmanFilter;

}  // namespace tools

#endif  // TOOLS__ERRSTATE_EXTENDED_KALMAN_HPP
