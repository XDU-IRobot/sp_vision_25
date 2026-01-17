#include "outpost_target.hpp"

namespace auto_aim
{

OutpostTarget::OutpostTarget(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig, const std::vector<double> &armor_heights)
: Target(armor, t, radius, armor_num, P0_dig)
{
  // 将状态向量扩展到13维: [x, vx, y, vy, z, vz, a, w, r, l, h, h1, h2]
  Eigen::VectorXd x_13d(13);
  x_13d.head(11) = ekf_.x;  // 复制前11维
  
  // 从配置初始化h1, h2（相对于中间装甲板的高度差）
  // 假设armor_heights = [h0, h1, h2]，我们让z=中间值，h1=装甲板1-中间，h2=装甲板2-中间
  if (armor_heights.size() == 3) {
    double h_mid = armor_heights[1];  // 中间装甲板高度作为参考
    x_13d[11] = armor_heights[0] - h_mid;  // h1: 装甲板0相对于中间的高度差
    x_13d[12] = armor_heights[2] - h_mid;  // h2: 装甲板2相对于中间的高度差
    
    // 调整z坐标到中间装甲板高度
    x_13d[4] = ekf_.x[4] - h_mid;
  } else {
    // 默认初始化为0
    x_13d[11] = 0.0;
    x_13d[12] = 0.0;
  }
  
  // 扩展协方差矩阵到13x13
  Eigen::MatrixXd P_13d = Eigen::MatrixXd::Zero(13, 13);
  P_13d.topLeftCorner(11, 11) = ekf_.P;
  P_13d(11, 11) = 1e-2;  // h1的初始方差
  P_13d(12, 12) = 1e-2;  // h2的初始方差
  
  // 更新ekf的状态和协方差
  ekf_.x = x_13d;
  ekf_.P = P_13d;
}

void OutpostTarget::predict(double dt)
{
  // 13维状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F(13, 13);
  F << 1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
       0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
       0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0,
       0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,
       0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0,
       0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,
       0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0,
       0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,
       0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,
       0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,
       0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,
       0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,
       0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1;
  // clang-format on

  // 前哨站专属过程噪声
  // auto [v1, v2] = get_process_noise();
  std::pair<double, double> process_noise = {10.0, 0.1};
  double v1 = process_noise.first;
  double v2 = process_noise.second;

  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  
  // 13维过程噪声协方差矩阵
  // clang-format off
  Eigen::MatrixXd Q(13, 13);
  Q << a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0, 0, 0,
       b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0, 0, 0,
            0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0, 0, 0,
            0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0, 0, 0,
            0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0, 0, 0,
            0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0, 0, 0,
            0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0, 0, 0,
            0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0, 0, 0,
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0, 0, 0,
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0, 0, 0,
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0, 0, 0,
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0, 0, 0,
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
  // clang-format on

  // 状态转移函数（角度限制）
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速限制
  if (this->convergened() && std::abs(this->ekf_.x[7]) > 2) {
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;
  }

  ekf_.predict(F, Q, f);
}

Eigen::Vector3d OutpostTarget::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  
  auto r = x[8];  // 前哨站所有装甲板使用相同半径
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  
  // 根据id选择对应的高度
  double armor_z;
  if (id == 0) {
    armor_z = x[4] + x[11];  // z + h1
  } else if (id == 1) {
    armor_z = x[4];          // z (中间装甲板)
  } else {  // id == 2
    armor_z = x[4] + x[12];  // z + h2
  }

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd OutpostTarget::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  
  auto r = x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);

  // 高度差的雅可比
  double dz_dh1 = (id == 0) ? 1.0 : 0.0;
  double dz_dh2 = (id == 2) ? 1.0 : 0.0;

  // 13列的雅可比矩阵：[x, vx, y, vy, z, vz, a, w, r, l, h, h1, h2]
  // clang-format off
  Eigen::MatrixXd H_armor_xyza(4, 13);
  H_armor_xyza << 1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr,     0,     0, 0,      0,
                  0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr,     0,     0, 0,      0,
                  0, 0, 0, 0, 1, 0,     0, 0,     0,     0,     0, dz_dh1, dz_dh2,
                  0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0, 0,      0;
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  
  // clang-format off
  Eigen::MatrixXd H_armor_ypda(4, 4);
  H_armor_ypda << H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0,
                  H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0,
                  H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0,
                                  0,                 0,                 0, 1;
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

// std::pair<double, double> OutpostTarget::get_process_noise() const
// {
//   return {10.0, 0.1};  // v1=10, v2=0.1 (前哨站专用)
// }

}  // namespace auto_aim