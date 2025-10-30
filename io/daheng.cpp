#include "daheng.hpp"

#include <stdexcept>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace io
{
Daheng::Daheng(const std::string & config_path)
: device_handle_(nullptr),
  payload_size_(0),
  quit_(false),
  ok_(false),
  queue_(1)
{
  // 从配置文件读取参数
  auto yaml = tools::load(config_path);
  
  // 基本参数
  gain_ = tools::read<double>(yaml, "gain");
  vid_pid_ = tools::read<std::string>(yaml, "vid_pid");
  
  // 调试开关
  debug_ = yaml["debug"] ? yaml["debug"].as<bool>() : false;
  
  // 相机详细设置
  if (yaml["camera_settings"]) {
    auto settings = yaml["camera_settings"];
    width_ = settings["width"].as<int>();
    height_ = settings["height"].as<int>();
    exposure_ = settings["exposure"].as<double>();
    auto_exposure_ = settings["auto_exposure"].as<bool>();
    fps_ = settings["fps"].as<int>();
    auto_white_balance_ = settings["auto_white_balance"].as<bool>();
    rgain_ = settings["rgain"].as<double>();
    bgain_ = settings["bgain"].as<double>();
    ggain_ = settings["ggain"].as<double>();
    time_offset_ = settings["time_offset"].as<int64_t>();
    auto_exp_change_ = settings["auto_exp_change"].as<bool>();
    max_exp_ = settings["max_exp"].as<int>();
    min_exp_ = settings["min_exp"].as<int>();
  } else {
    // 默认值
    width_ = 1920;
    height_ = 1200;
    exposure_ = 2100;
    auto_exposure_ = false;
    fps_ = 168;
    auto_white_balance_ = true;
    rgain_ = 0.0;
    bgain_ = 0.0;
    ggain_ = 0.0;
    time_offset_ = 45500000;
    auto_exp_change_ = false;
    max_exp_ = 3000;
    min_exp_ = 200;
  }

  try_open();

  // 守护线程
  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("Daheng daemon thread started");
    while (!quit_) {
      std::this_thread::sleep_for(100ms);

      if (ok_) continue;

      if (capture_thread_.joinable()) capture_thread_.join();

      close();
      try_open();
    }
    tools::logger()->info("Daheng daemon thread stopped");
  }};

  tools::logger()->info("Daheng camera constructor completed with VID:PID = {}", vid_pid_);
}

Daheng::~Daheng()
{
  quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  if (capture_thread_.joinable()) capture_thread_.join();
  close();
  
  // 关闭库
  GXCloseLib();
  tools::logger()->info("Daheng camera destructed");
}

void Daheng::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

void Daheng::try_open()
{
  tools::logger()->info("Daheng::try_open() called");
  try {
    open();
    ok_ = true;
    tools::logger()->info("Daheng camera opened successfully");
  } catch (const std::exception & e) {
    tools::logger()->warn("Failed to open Daheng camera: {}", e.what());
    ok_ = false;
  }
}

void Daheng::open()
{
  tools::logger()->info("Daheng::open() called - initializing Daheng library");
  
  // 初始化库
  GX_STATUS status = GXInitLib();
  if (status != GX_STATUS_SUCCESS) {
    throw std::runtime_error("Failed to initialize Daheng library");
  }

  tools::logger()->info("Daheng library initialized successfully");

  // 枚举设备
  uint32_t device_num = 0;
  status = GXUpdateDeviceList(&device_num, 1000);
  if (status != GX_STATUS_SUCCESS) {
    throw std::runtime_error("Failed to update device list");
  }
  
  tools::logger()->info("Found {} Daheng camera(s)", device_num);
  
  if (device_num <= 0) {
    throw std::runtime_error("No Daheng camera found");
  }

  // 打开第一个设备 (可以根据vid_pid进行筛选)
  status = GXOpenDeviceByIndex(1, &device_handle_);
  if (status != GX_STATUS_SUCCESS) {
    throw std::runtime_error("Failed to open Daheng camera device");
  }

  // 获取图像大小
  int64_t width_val, height_val;
  status = GXGetInt(device_handle_, GX_INT_WIDTH, &width_val);
  if (status != GX_STATUS_SUCCESS) {
    throw std::runtime_error("Failed to get image width");
  }
  width_ = static_cast<int>(width_val);

  status = GXGetInt(device_handle_, GX_INT_HEIGHT, &height_val);
  if (status != GX_STATUS_SUCCESS) {
    throw std::runtime_error("Failed to get image height");
  }
  height_ = static_cast<int>(height_val);

  // 获取图像缓冲区大小
  status = GXGetInt(device_handle_, GX_INT_PAYLOAD_SIZE, &payload_size_);
  if (status != GX_STATUS_SUCCESS) {
    throw std::runtime_error("Failed to get payload size");
  }

  // 设置相机参数
  set_camera_params();

  // 开始采集
  status = GXStreamOn(device_handle_);
  if (status != GX_STATUS_SUCCESS) {
    throw std::runtime_error("Failed to start stream");
  }

  // 启动采集线程
  capture_thread_ = std::thread{[this] {
    tools::logger()->info("Daheng capture thread started");
    
    unsigned char * image_buffer = new unsigned char[payload_size_];
    unsigned char * rgb_buffer = new unsigned char[width_ * height_ * 3];
    
    while (!quit_ && ok_) {
      GX_FRAME_DATA frame_data;
      frame_data.pImgBuf = image_buffer;
      
      GX_STATUS status = GXGetImage(device_handle_, &frame_data, 100);
      
      if (status == GX_STATUS_SUCCESS && frame_data.nStatus == GX_FRAME_STATUS_SUCCESS) {
        // 转换为RGB格式
        DX_BAYER_CONVERT_TYPE convert_type = RAW2RGB_NEIGHBOUR;
        // 尝试匹配像素格式所对应的 Bayer 布局；默认 BAYERRG
        DX_PIXEL_COLOR_FILTER color_filter = BAYERRG;
        // 注意：无法直接从设备句柄读取我们设定的像素格式，这里按常见顺序匹配
        // 若上面像素格式设置为 RG8/GR8/GB8/BG8，则这里分别使用 BAYERRG/BAYERGR/BAYERGB/BAYERBG
        bool flip = false;
        
        if (debug_) {
          static int frame_count = 0;
          if (frame_count % 100 == 0) {  // 每100帧输出一次调试信息
            const char *bayer_name = "BAYERRG";
            if (color_filter == BAYERGR) bayer_name = "BAYERGR";
            else if (color_filter == BAYERGB) bayer_name = "BAYERGB";
            else if (color_filter == BAYERBG) bayer_name = "BAYERBG";
            tools::logger()->info("Processing frame {}, using Bayer filter: {}", frame_count, bayer_name);
          }
          frame_count++;
        }
        
        DxRaw8toRGB24(
          (unsigned char*)frame_data.pImgBuf,
          rgb_buffer,
          width_,
          height_,
          convert_type,
          color_filter,
          flip
        );
        
        // 创建OpenCV Mat - 直接创建为BGR格式
        cv::Mat img_rgb(height_, width_, CV_8UC3, rgb_buffer);
        cv::Mat img_bgr;
        cv::cvtColor(img_rgb, img_bgr, cv::COLOR_RGB2BGR);
        
        CameraData data;
        data.img = img_bgr.clone();
        data.timestamp = std::chrono::steady_clock::now();
        
        queue_.push(data);
      }
      
      std::this_thread::sleep_for(1ms);
    }
    
    delete[] image_buffer;
    delete[] rgb_buffer;
    
    tools::logger()->info("Daheng capture thread stopped");
  }};
}

void Daheng::close()
{
  if (device_handle_) {
    // 停止采集
    GXStreamOff(device_handle_);
    
    // 关闭设备
    GXCloseDevice(device_handle_);
    device_handle_ = nullptr;
  }
  
  ok_ = false;
  tools::logger()->info("Daheng camera closed");
}

void Daheng::set_camera_params()
{
  GX_STATUS status;
  
  if (debug_) {
    tools::logger()->info("Setting Daheng camera parameters:");
    tools::logger()->info("  Size: {}x{}", width_, height_);
    tools::logger()->info("  Exposure: {}", exposure_);
    tools::logger()->info("  Gain: {}", gain_);
    tools::logger()->info("  FPS: {}", fps_);
  }
  
  // 设置图像尺寸
  status = GXSetInt(device_handle_, GX_INT_WIDTH, width_);
  if (status != GX_STATUS_SUCCESS && debug_) {
    tools::logger()->warn("Failed to set width to {}", width_);
  }
  
  status = GXSetInt(device_handle_, GX_INT_HEIGHT, height_);
  if (status != GX_STATUS_SUCCESS && debug_) {
    tools::logger()->warn("Failed to set height to {}", height_);
  }
  
  // 设置曝光模式
  if (auto_exposure_) {
    status = GXSetEnum(device_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    if (status != GX_STATUS_SUCCESS && debug_) {
      tools::logger()->warn("Failed to set auto exposure");
    }
    
    // 设置曝光范围
    status = GXSetFloat(device_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, max_exp_);
    status = GXSetFloat(device_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, min_exp_);
  } else {
    status = GXSetEnum(device_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    status = GXSetFloat(device_handle_, GX_FLOAT_EXPOSURE_TIME, exposure_);
    if (status != GX_STATUS_SUCCESS && debug_) {
      tools::logger()->warn("Failed to set exposure time to {}", exposure_);
    }
  }
  
  // 设置增益
  status = GXSetFloat(device_handle_, GX_FLOAT_GAIN, gain_);
  if (status != GX_STATUS_SUCCESS && debug_) {
    tools::logger()->warn("Failed to set gain to {}", gain_);
  }
  
  // 设置白平衡
  if (auto_white_balance_) {
    status = GXSetEnum(device_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
  } else {
    status = GXSetEnum(device_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
    // 设置RGB增益 - 需要先选择通道再设置值
    if (rgain_ > 0) {
      status = GXSetEnum(device_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
      status = GXSetFloat(device_handle_, GX_FLOAT_BALANCE_RATIO, rgain_);
    }
    if (bgain_ > 0) {
      status = GXSetEnum(device_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
      status = GXSetFloat(device_handle_, GX_FLOAT_BALANCE_RATIO, bgain_);
    }
    if (ggain_ > 0) {
      status = GXSetEnum(device_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
      status = GXSetFloat(device_handle_, GX_FLOAT_BALANCE_RATIO, ggain_);
    }
  }
  
  // 设置触发模式为连续模式
  status = GXSetEnum(device_handle_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
  if (status != GX_STATUS_SUCCESS && debug_) {
    tools::logger()->warn("Failed to set trigger mode");
  }
  
  // 设置采集模式为连续采集
  status = GXSetEnum(device_handle_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
  if (status != GX_STATUS_SUCCESS && debug_) {
    tools::logger()->warn("Failed to set acquisition mode");
  }
  
  // 设置像素格式为 Bayer8；若失败则尝试其他排列
  GX_PIXEL_FORMAT_ENTRY try_formats[] = {
    GX_PIXEL_FORMAT_BAYER_RG8,
    GX_PIXEL_FORMAT_BAYER_GR8,
    GX_PIXEL_FORMAT_BAYER_GB8,
    GX_PIXEL_FORMAT_BAYER_BG8
  };
  
  const char *fmt_names[] = {"BAYER_RG8", "BAYER_GR8", "BAYER_GB8", "BAYER_BG8"};
  int chosen = -1;
  for (int i = 0; i < 4; ++i) {
    status = GXSetEnum(device_handle_, GX_ENUM_PIXEL_FORMAT, try_formats[i]);
    if (status == GX_STATUS_SUCCESS) { chosen = i; break; }
  }
  if (chosen == -1) {
    if (debug_) tools::logger()->warn("Failed to set any Bayer8 pixel format");
    // keep default from camera
  } else if (debug_) {
    tools::logger()->info("Pixel format set to {}", fmt_names[chosen]);
  }
  
  if (debug_) {
    tools::logger()->info("Daheng camera parameters configured successfully");
  }
}

void GX_STDC Daheng::frame_callback(GX_FRAME_CALLBACK_PARAM * frame_data)
{
  // 这是回调函数的实现，目前使用主动获取图像的方式
  // 如果需要使用回调方式，可以在这里实现
}

}  // namespace io
