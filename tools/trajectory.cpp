#include "trajectory.hpp"

#include <cmath>

namespace tools
{
constexpr double g = 9.7833;

Trajectory::Trajectory(const double v0, const double d, const double h)
{
  auto a = g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  auto delta = b * b - 4 * a * c;

  if (delta < 0) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  auto t_1 = d / (v0 * std::cos(pitch_1));
  auto t_2 = d / (v0 * std::cos(pitch_2));

  // if (h > 0) {
  //   // 目标在上方，必须选择正角度（抬头）
  //   if (pitch_1 > 0 && pitch_2 > 0) {
  //     // 两个都是正的，选飞行时间短的
  //     pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  //     fly_time = (t_1 < t_2) ? t_1 : t_2;
  //   } else if (pitch_1 > 0) {
  //     // 只有pitch_1是正的，必须选它
  //     pitch = pitch_1;
  //     fly_time = t_1;
  //   } else if (pitch_2 > 0) {
  //     // 只有pitch_2是正的，必须选它
  //     pitch = pitch_2;
  //     fly_time = t_2;
  //   } else {
  //     // 两个都是负的（异常情况），兜底选时间短的
  //     pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  //     fly_time = (t_1 < t_2) ? t_1 : t_2;
  //   }
  // } else {
  //   // 目标在下方或平视，保持原逻辑（你说下方时跟得很正常）
  //   pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  //   fly_time = (t_1 < t_2) ? t_1 : t_2;
  // }
  


  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  fly_time = (t_1 < t_2) ? t_1 : t_2;
}

}  // namespace tools