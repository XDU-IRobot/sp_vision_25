#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace tools {
class MarkerPublisher;
}

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);
  virtual ~Target() = default;

  virtual void predict(std::chrono::steady_clock::time_point t);
  virtual void predict(double dt);
  virtual void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  virtual std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

  /* 可视化相关 */
  virtual void set_marker_publisher(tools::MarkerPublisher* marker_pub);
  virtual void visualize(int base_id) const;

protected:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  virtual void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  virtual Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  virtual Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;

  // virtual std::pair<double, double> get_process_noise() const=0;

  tools::MarkerPublisher* marker_pub_ = nullptr;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP