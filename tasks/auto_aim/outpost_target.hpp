#ifndef AUTO_AIM__OUTPOST_TARGET_HPP
#define AUTO_AIM__OUTPOST_TARGET_HPP

#include "tasks/auto_aim/target.hpp"
#include <vector>
#include <cmath>
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
  /**
   * @brief 前哨站目标类 - 继承自Target，使用13维状态向量
   * 
   * 状态向量: [x, vx, y, vy, z, vz, a, w, r, l, h, h1, h2]
   * - x, y, z: 旋转中心坐标（z是中间装甲板高度）
   * - vx, vy, vz: 速度
   * - a: 角度
   * - w: 角速度
   * - r: 半径
   * - l: 未使用（保持兼容）
   * - h: 未使用（保持兼容）
   * - h1: 装甲板1相对于装甲板0的高度差
   * - h2: 装甲板2相对于装甲板0的高度差
   */
  class OutpostTarget : public Target
  {
  public:
    /**
     * @brief 构造函数
     * @param armor 首次观测到的装甲板
     * @param t 观测时间
     * @param radius 旋转半径
     * @param armor_num 装甲板数量（前哨站固定为3）
     * @param P0_dig 初始协方差对角线元素（13维）
     * @param armor_heights 装甲板高度配置（用于初始化h1, h2）
     */
    OutpostTarget(const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
                  Eigen::VectorXd P0_dig, const std::vector<double> &armor_heights);

    // 重写predict以使用13维状态转移
    void predict(double dt) override;

    // 重载update以处理13维状态
    void update(const Armor & armor) override;
    // 加入匹配函数： 根据对应的z坐标和高度差匹配到相应的装甲板id
    int match_armor_id(double z_obs) const;

    // 重写可视化方法
    void visualize(int base_id) const override;
  protected:
    // 重写观测模型：装甲板xyz坐标计算（使用状态变量h1, h2）
    Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const override;
    
    // 重写观测雅可比矩阵（13列）
    Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const override;

    // // 获取前哨站专属过程噪声参数
    // virtual std::pair<double, double> get_process_noise() const override;
  };
}  // namespace auto_aim

#endif  // AUTO_AIM__OUTPOST_TARGET_HPP