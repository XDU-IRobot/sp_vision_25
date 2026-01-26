#include "outpost_target.hpp"
#include "tools/marker_publisher.hpp"
#include "tools/math_tools.hpp"
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
  
  // -z (x_13d[4]) = 第一块装甲板的高度，将第一块装甲板观测的z坐标作为前哨站center的高度即可在第一帧完成初始化
  // -h1,h2 = 其他两块装甲板相对于第一块装甲板的高度差
  if (armor_heights.size() == 3) {
    // todo: 使用测量高度完成装甲板的先验

    x_13d[11] = armor_heights[1] - armor_heights[0];  // h1: 装甲板1相对于装甲板0的高度差
    x_13d[12] = armor_heights[2] - armor_heights[0];  // h2: 装甲板2相对于装甲板0的高度差
    tools::logger()->info("OutpostTarget init: z={:.3f}, h1={:.3f}, h2={:.3f}",
                         x_13d[4], x_13d[11], x_13d[12]);
  } else {
    // 默认初始化为0
    x_13d[11] = 0.1;
    x_13d[12] = -0.1;
  }
  
  // 扩展协方差矩阵到13x13
  Eigen::MatrixXd P_13d = P0_dig.asDiagonal();
  ekf_ = tools::ExtendedKalmanFilter(x_13d, P_13d);
  
  // 初始化：记录第一块观测到的装甲板高度
  observed_heights_.push_back(armor.xyz_in_world[2]);
  tools::logger()->info("OutpostTarget: First armor height recorded: {:.3f}", armor.xyz_in_world[2]);
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
  Q << a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0,    0,    0,
       b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0,    0,    0,
            0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0,    0,    0,
            0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0,    0,    0,
            0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0,    0,    0,
            0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0,    0,    0,
            0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0,    0,    0,
            0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0,    0,    0,
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0,    0,    0,
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0,    0,    0,
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0,    0,    0,
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0, 1e-6,    0,  // h1微小噪声
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0,    0, 1e-6;  // h2微小噪声
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

int OutpostTarget::match_armor_id(double z_obs) const
{
  // 计算观测高度与各装甲板预测高度的差值
  double z0 = ekf_.x[4];                     // 装甲板0的预测高度
  double z1 = ekf_.x[4] + ekf_.x[11];       // 装甲板1的预测高度
  double z2 = ekf_.x[4] + ekf_.x[12];       // 装甲板2的预测高度

  double diff0 = std::abs(z_obs - z0);
  double diff1 = std::abs(z_obs - z1);
  double diff2 = std::abs(z_obs - z2);

  // 调试输出
  // tools::logger()->trace(
  //   "match_armor_id: z_obs={:.3f}, z0={:.3f}, z1={:.3f}, z2={:.3f}, "
  //   "diff=[{:.3f}, {:.3f}, {:.3f}]",
  //   z_obs, z0, z1, z2, diff0, diff1, diff2
  // );

  // 找到最小差值对应的装甲板id
  if (diff0 <= diff1 && diff0 <= diff2) {
    return 0;
  } else if (diff1 <= diff0 && diff1 <= diff2) {
    return 1;
  } else {
    return 2;
  }
}

void OutpostTarget::update(const Armor & armor)
{
  double h1_variance = ekf_.P(11, 11);
  double h2_variance = ekf_.P(12, 12);
  
  bool h_converged = (h1_variance < 0.01) && (h2_variance < 0.01) && 
                     (std::abs(ekf_.x[11]) > 0.03 || std::abs(ekf_.x[12]) > 0.03);
  double z_obs = armor.xyz_in_world[2];
  int height_matched_id = match_armor_id(z_obs); 
  // 在±1范围内综合评估
  std::vector<Eigen::Vector4d> xyza_list = armor_xyza_list();
  double min_error = 1e10;
  int final_id = height_matched_id;
  
  for (int offset = -1; offset <= 1; offset++) {
    int check_id = (height_matched_id + offset + armor_num_) % armor_num_;
    const auto & xyza = xyza_list[check_id];
    
    double angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3]));
    double height_error = std::abs(z_obs - xyza[2]) * (h_converged ? 2.0 : 0.5);
    
    double total_error = angle_error + height_error;
    
    if (total_error < min_error) {
      min_error = total_error;
      final_id = check_id;
    }
  }
  
  // 跳变检测
  if (final_id != 0) jumped = true;
  if (final_id != last_id) is_switch_ = true;
  else is_switch_ = false;
  if (is_switch_) switch_count_++;
  
  last_id = final_id;
  update_count_++;
  
  // EKF 更新
  update_ypda(armor, final_id);
}

std::vector<Eigen::Vector4d> OutpostTarget::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> armor_xyza_list;
  for (int id = 0; id < armor_num_; id++) {
    auto angle = tools::limit_rad(ekf_.x[6] + id * 2 * CV_PI / armor_num_);
    auto r = ekf_.x[8];
    auto armor_x = ekf_.x[0] - r * std::cos(angle);
    auto armor_y = ekf_.x[2] - r * std::sin(angle);
    double armor_z;
    if (id == 0) {
      armor_z = ekf_.x[4];
    } else if (id == 1) {
      armor_z = ekf_.x[4] + ekf_.x[11];
    } else {  // id == 2
      armor_z = ekf_.x[4] + ekf_.x[12];
    }
    armor_xyza_list.push_back({armor_x, armor_y, armor_z, angle});
  }
  return armor_xyza_list;
}
Eigen::Vector3d OutpostTarget::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{ 
  auto angle = tools::limit_rad( x[6] + id * 2 * CV_PI / armor_num_);
  
  auto r = x[8];  // 前哨站所有装甲板使用相同半径
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  
  // 根据id映射到对应的高度
  double armor_z;
  if (id == 0) {
    armor_z = x[4];
  } else if (id == 1) {
    armor_z = x[4] + x[11];         //第二块观测装甲板
  } else {  // id == 2
    armor_z = x[4] + x[12];  // 第三块观测装甲板
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
  double dz_dh1 = (id == 1) ? 1.0 : 0.0;
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
void OutpostTarget::visualize(int base_id) const
{
  if (!marker_pub_) return;

  // 获取状态
  Eigen::Vector3d center(ekf_.x[0], ekf_.x[2], ekf_.x[4]);
  double radius = ekf_.x[8];
  double vyaw = ekf_.x[7];

  // ============================================
  // 1. 可视化旋转信息
  // ============================================

  marker_pub_->publishSphereMarker(
    "world", base_id, center, 0.08, 255, 255, 0
  );

  marker_pub_->publishCircleMarker(
    "world", base_id + 1, center, radius, 255, 255, 0, 0.3
  );

  if (std::abs(vyaw) > 0.1) {
    Eigen::Vector3d arrow_end = center;
    arrow_end[2] += vyaw * 0.2;
    marker_pub_->publishArrowMarker(
      "world", base_id + 2, center, arrow_end, 255, 0, 255
    );
  }

  // ============================================
  // 2. 可视化三个装甲板
  // ============================================
  
  std::vector<Eigen::Vector4d> armor_list = armor_xyza_list();
  
  for (size_t i = 0; i < armor_list.size(); i++) {
    const auto& xyza = armor_list[i];
    Eigen::Vector3d armor_pos(xyza[0], xyza[1], xyza[2]);
    double armor_yaw = xyza[3];
    
    Eigen::Quaterniond q(Eigen::AngleAxisd(armor_yaw, Eigen::Vector3d::UnitZ()));
    
    // 前哨站装甲板尺寸
    double width = (armor_type == ArmorType::small) ? 0.135 : 0.23;
    double height = 0.125;
    
    // 不同装甲板用不同颜色区分
    int r, g, b;
    if (i == static_cast<size_t>(last_id)) {
      r = 0; g = 255; b = 0;  // 当前匹配：绿色
    } else {
      switch (i) {
        case 0: r = 255; g = 100; b = 100; break;  // 红
        case 1: r = 100; g = 100; b = 255; break;  // 蓝
        case 2: r = 255; g = 255; b = 100; break;  // 黄
        default: r = 150; g = 150; b = 150; break;
      }
    }

    marker_pub_->publishArmorMarker(
      "world", base_id + 100 + i, armor_pos, q, width, height, r, g, b
    );
  }

  // ============================================
  // 3. 可视化三角形结构（使用 ARROW 替代 LINE）
  // ============================================
  
  if (armor_list.size() == 3) {
    for (size_t i = 0; i < 3; i++) {
      size_t next = (i + 1) % 3;
      
      Eigen::Vector3d p1(armor_list[i][0], armor_list[i][1], armor_list[i][2]);
      Eigen::Vector3d p2(armor_list[next][0], armor_list[next][1], armor_list[next][2]);

      // ✅ 使用箭头显示连线（去掉箭头头部，只留线段）
      marker_pub_->publishArrowMarker(
        "world", base_id + 200 + i, p1, p2, 255, 255, 255, 0.5
      );
    }
  }
}


// std::pair<double, double> OutpostTarget::get_process_noise() const
// {
//   return {10.0, 0.1};  // v1=10, v2=0.1 (前哨站专用)
// }

}  // namespace auto_aim