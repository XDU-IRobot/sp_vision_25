#include "yolo.hpp"

#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include "tools/logger.hpp"

// 条件引入OpenVINO实现
#ifdef ENABLE_OPENVINO
#include "yolos/yolov5.hpp"
#include "yolos/yolo11.hpp"
#include "yolos/yolov8.hpp"
#include "yolos/yolo26.hpp"
#endif

// 条件引入TensorRT实现
#ifdef ENABLE_TENSORRT
#include "Point4model.hpp"
#include "yolos/yolo11_trt.hpp"
#include "yolos/yolo26_trt.hpp"
#include "yolos/yolov8_trt.hpp"
#endif

namespace auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug)
{
  // 1. 检查至少有一个后端可用
  #if !defined(ENABLE_OPENVINO) && !defined(ENABLE_TENSORRT)
    throw std::runtime_error(
      "=== No Inference Backend Available ===\n"
      "No inference backend compiled!\n"
      "Please compile with OpenVINO or TensorRT support."
    );
  #endif

  // 2. 读取配置
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();

  // 3. 读取后端配置（必须指定，不提供默认值）
  if (!yaml["inference_backend"]) {
    throw std::runtime_error(
      "=== Configuration Error ===\n"
      "Missing 'inference_backend' in config file!\n"
      "Please add one of the following to your YAML config:\n"
      "  inference_backend: tensorrt  # Use TensorRT\n"
      "  inference_backend: openvino  # Use OpenVINO\n"
    );
  }

  std::string backend = yaml["inference_backend"].as<std::string>();

  // 4. 验证后端可用性
  bool use_tensorrt = false;
  if (backend == "tensorrt") {
    #ifdef ENABLE_TENSORRT
      use_tensorrt = true;
    #else
      throw std::runtime_error(
        "=== TensorRT Not Available ===\n"
        "TensorRT backend requested but not compiled!\n"
        "Please either:\n"
        "  1. Install TensorRT and recompile\n"
        "  2. Change config to: inference_backend: openvino\n"
      );
    #endif
  } else if (backend == "openvino") {
    #ifdef ENABLE_OPENVINO
      use_tensorrt = false;
    #else
      throw std::runtime_error(
        "=== OpenVINO Not Available ===\n"
        "OpenVINO backend requested but not compiled!\n"
        "Please either:\n"
        "  1. Install OpenVINO and recompile\n"
        "  2. Change config to: inference_backend: tensorrt\n"
      );
    #endif
  } else {
    throw std::runtime_error(
      "=== Invalid Backend Configuration ===\n"
      "Invalid inference_backend: '" + backend + "'!\n"
      "Valid options are:\n"
      "  - tensorrt\n"
      "  - openvino\n"
    );
  }

  // 5. 根据yolo_name和backend创建实例

  // ========== TensorRT 后端 ==========
  if (use_tensorrt) {
    #ifdef ENABLE_TENSORRT
      if (yolo_name == "yolo11") {
        tools::logger()->info("[YOLO] backend=tensorrt model=yolo11 impl=YOLO11_TRT config={}", config_path);
        yolo_ = std::make_unique<YOLO11_TRT>(config_path, debug);
        return;
      }
      else if (yolo_name == "yolov8") {
        tools::logger()->info("[YOLO] backend=tensorrt model=yolov8 impl=YOLOV8_TRT config={}", config_path);
        yolo_ = std::make_unique<YOLOV8_TRT>(config_path, debug);
        return;
      }
      else if (yolo_name == "yolo26") {
        tools::logger()->info("[YOLO] backend=tensorrt model=yolo26 impl=YOLO26_TRT config={}", config_path);
        yolo_ = std::make_unique<YOLO26_TRT>(config_path, debug);
        return;
      }
      else if (yolo_name == "point4") {
        tools::logger()->info("[YOLO] backend=tensorrt model=point4 impl=Point4ModelTRT config={}", config_path);
        yolo_ = std::make_unique<Point4ModelTRT>(config_path, debug);
        return;
      }

      throw std::runtime_error(
        "=== TensorRT Model Not Supported ===\n"
        "Model '" + yolo_name + "' is not supported with TensorRT backend!\n"
        "TensorRT supports: yolo11, yolov8, yolo26, point4\n"
      );
    #endif
  }

  // ========== OpenVINO 后端 ==========
  else {
    #ifdef ENABLE_OPENVINO
      if (yolo_name == "yolov5") {
        tools::logger()->info("[YOLO] backend=openvino model=yolov5 impl=YOLOV5 config={}", config_path);
        yolo_ = std::make_unique<YOLOV5>(config_path, debug);
        return;
      }
      else if (yolo_name == "yolov8") {
        tools::logger()->info("[YOLO] backend=openvino model=yolov8 impl=YOLOV8 config={}", config_path);
        yolo_ = std::make_unique<YOLOV8>(config_path, debug);
        return;
      }
      else if (yolo_name == "yolo11") {
        tools::logger()->info("[YOLO] backend=openvino model=yolo11 impl=YOLO11 config={}", config_path);
        yolo_ = std::make_unique<YOLO11>(config_path, debug);
        return;
      }
      else if (yolo_name == "yolo26") {
        tools::logger()->info("[YOLO] backend=openvino model=yolo26 impl=YOLO26 config={}", config_path);
        yolo_ = std::make_unique<YOLO26>(config_path, debug);
        return;
      }

      throw std::runtime_error(
        "=== OpenVINO Model Not Supported ===\n"
        "Model '" + yolo_name + "' is not supported with OpenVINO backend!\n"
        "OpenVINO supports: yolov5, yolov8, yolo11, yolo26\n"
      );
    #endif
  }
}

std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim
