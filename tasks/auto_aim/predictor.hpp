#ifndef AUTO_AIM__PREDICTOR_HPP
#define AUTO_AIM__PREDICTOR_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>
#include <cmath>

#include "armor.hpp"
#include "solver.hpp"

namespace auto_aim
{

class Predictor
{
public:
  // 维护一个预测结果结构体来解耦合
  struct PredictResult {
  bool valid;                       // 是否有足够数据/预测成功
  std::chrono::steady_clock::time_point stamp;  // 预测对应的时间（例如 t 或前瞻后的时间）
  Eigen::Vector3d position;         // 预测的装甲中心坐标（世界系或云台系）
  Eigen::Vector3d velocity;         // 当前估计的速度
  double yaw;                       // 若你同时预测装甲朝向
  double yaw_rate;
  double horizon;                   // 这次预测前瞻的时间长度（秒）
};
  //考虑维护一个state来记录当前状态
  std::string state_();
  Predictor(const std::string & config_path, Solver & solver);
  // 暴露给外部的接口
  PredictResult track(const Armor & armor, std::chrono::steady_clock::time_point t);
  
private:
  std::string state_, pre_state_;
  std::chrono::steady_clock::time_point last_timestamp_;
  Solver & solver_;
  Color enemy_color_;
  // 考虑是否要加入最小连续帧，以避免误识别
  double pos_alpha_;        // 位置 EWMA 系数
  double vel_alpha_;        // 速度 EWMA 系数
  Eigen::Vector3d last_pos;
  Eigen::Vector3d pos_ewma;
  Eigen::Vector3d vel_ewma;
  /*todo :使用球坐标建模装甲板位置估计    */

  void ingest(const Armor & armor, std::chrono::steady_clock::time_point t); //update target
  Eigen::Vector3d diff_velocity(const Eigen::Vector3d & curr_pos,double dt);
  void update_ewma(Eigen::Vector3d & curr_pos, Eigen::Vector3d & vel);
  Eigen::Vector3d predict_pos(double horizon_sec) const;
  bool ready() const;
};
}

#endif
