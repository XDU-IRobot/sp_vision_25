#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }";

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
}

void capture_loop(
  const std::string & config_path, const std::string & output_folder,
  int pattern_cols, int pattern_rows)
{
  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  cv::Size pattern_size(pattern_cols, pattern_rows);
  int count = 0;
  int failed_count = 0;

  tools::logger()->info("=== 棋盘格标定数据采集 ===");
  tools::logger()->info("标定板尺寸: {}x{} 内角点", pattern_cols, pattern_rows);
  tools::logger()->info("按 's' 键保存, 按 'q' 键退出");

  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = cboard.imu_at(timestamp);

    // 在图像上显示欧拉角
    auto img_with_info = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
    tools::draw_text(img_with_info, fmt::format("IMU Z(Yaw): {:.1f}", zyx[0]), {10, 30}, {255, 255, 0});
    tools::draw_text(img_with_info, fmt::format("IMU Y(Pitch): {:.1f}", zyx[1]), {10, 60}, {255, 255, 0});
    tools::draw_text(img_with_info, fmt::format("IMU X(Roll): {:.1f}", zyx[2]), {10, 90}, {255, 255, 0});

    // 检测棋盘格角点
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(
      img, pattern_size, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
      // 亚像素精化（提高精度）
      cv::Mat gray;
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
      cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

      // 显示检测到的角点
      cv::drawChessboardCorners(img_with_info, pattern_size, corners, found);
      tools::draw_text(img_with_info, "Chessboard OK!", {10, 120}, {0, 255, 0}, 1.0, 2);
      failed_count = 0;
    } else {
      failed_count++;
      tools::draw_text(img_with_info, fmt::format("NOT found ({})", failed_count),
                       {10, 120}, {0, 0, 255}, 0.8, 2);
    }

    // 显示已采集数量
    tools::draw_text(img_with_info, fmt::format("Captured: {}", count),
                     {10, img_with_info.rows - 20}, {255, 255, 255}, 1.0, 2);

    cv::resize(img_with_info, img_with_info, {}, 0.5, 0.5);
    cv::imshow("Chessboard Capture - 's' to save, 'q' to quit", img_with_info);

    auto key = cv::waitKey(1);
    if (key == 'q') {
      break;
    } else if (key == 's') {
      if (!found) {
        tools::logger()->warn("未检测到棋盘格，无法保存！");
        continue;
      }

      // 保存图片和四元数
      count++;
      auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
      auto q_path = fmt::format("{}/{}.txt", output_folder, count);
      cv::imwrite(img_path, img);
      write_q(q_path, q);
      tools::logger()->info("[{}] Saved to {} (corners: {})", count, output_folder, corners.size());
    }
  }

  tools::logger()->info("采集完成！共 {} 组数据", count);
  if (count < 10) {
    tools::logger()->warn("建议至少采集10-15组数据");
  }
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");

  // 新建输出文件夹
  std::filesystem::create_directory(output_folder);

  // 读取标定板尺寸
  auto yaml = YAML::LoadFile(config_path);
  int pattern_cols = yaml["pattern_cols"].as<int>();
  int pattern_rows = yaml["pattern_rows"].as<int>();

  // 主循环，保存图片和对应四元数
  capture_loop(config_path, output_folder, pattern_cols, pattern_rows);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
