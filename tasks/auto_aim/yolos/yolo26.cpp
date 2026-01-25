// #include "yolo26.hpp"

// #include <fmt/chrono.h>
// #include <yaml-cpp/yaml.h>

// #include <algorithm>
// #include <chrono>
// #include <filesystem>
// #include <fstream>
// #include <future>
// #include <limits>
// #include <thread>
// #include <vector>

// #include <sched.h>

// #include "tools/img_tools.hpp"
// #include "tools/logger.hpp"

// namespace auto_aim
// {

// namespace
// {
// struct BigCoreInfo {
//   std::vector<int> ids;
//   bool core_type_available = false;
//   long max_freq = -1;
// };

// BigCoreInfo get_big_core_info()
// {
//   BigCoreInfo info;
//   std::vector<int> big_cores;
//   const unsigned int cpu_count = std::thread::hardware_concurrency();
//   for (unsigned int cpu = 0; cpu < cpu_count; ++cpu) {
//     std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/topology/core_type";
//     std::ifstream file(path);
//     int core_type = -1;
//     if (file.good() && (file >> core_type)) {
//       info.core_type_available = true;
//       if (core_type != 0) {
//         big_cores.push_back(static_cast<int>(cpu));
//       }
//     }
//   }
//   if (!info.core_type_available) {
//     long max_freq = -1;
//     std::vector<long> freqs(cpu_count, -1);
//     for (unsigned int cpu = 0; cpu < cpu_count; ++cpu) {
//       std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/cpufreq/cpuinfo_max_freq";
//       std::ifstream file(path);
//       long freq = -1;
//       if (file.good() && (file >> freq)) {
//         freqs[cpu] = freq;
//         max_freq = std::max(max_freq, freq);
//       }
//     }
//     info.max_freq = max_freq;
//     if (max_freq > 0) {
//       for (unsigned int cpu = 0; cpu < cpu_count; ++cpu) {
//         if (freqs[cpu] == max_freq) {
//           big_cores.push_back(static_cast<int>(cpu));
//         }
//       }
//     }
//   }
//   info.ids = std::move(big_cores);
//   return info;
// }

// bool bind_current_thread_to_cores(const std::vector<int> & cores)
// {
//   if (cores.empty()) {
//     return false;
//   }
//   cpu_set_t cpuset;
//   CPU_ZERO(&cpuset);
//   for (int cpu : cores) {
//     CPU_SET(cpu, &cpuset);
//   }
//   return sched_setaffinity(0, sizeof(cpuset), &cpuset) == 0;
// }
// }  // namespace
// YOLO26::YOLO26(const std::string & config_path, bool debug)
// : debug_(debug), detector_(config_path, false), current_request_idx_(0), first_frame_(true), has_pending_postprocess_(false)
// {
//   auto yaml = YAML::LoadFile(config_path);

//   model_path_ = yaml["yolo26_model_path"].as<std::string>();
//   device_ = yaml["device"].as<std::string>();
//   binary_threshold_ = yaml["threshold"].as<double>();
//   min_confidence_ = yaml["min_confidence"].as<double>();

//   // 读取异步推理配置（默认false）
//   use_async_inference_ = yaml["use_async_inference"].as<bool>(false);

//   // 大核识别（同步/异步都可用），用于绑定和推理线程配置
//   std::vector<int> big_cores;
//   const auto big_info = get_big_core_info();
//   big_cores = big_info.ids;
//   const int cpu_cores = static_cast<int>(std::thread::hardware_concurrency());
//   tools::logger()->warn(
//     "[YOLO26] CPU cores: {}, big cores found: {}, core_type: {}, max_freq: {}",
//     std::thread::hardware_concurrency(), big_cores.size(),
//     big_info.core_type_available ? "available" : "unavailable",
//     big_info.max_freq);
//   if (!big_cores.empty()) {
//     std::string core_list;
//     for (size_t i = 0; i < big_cores.size(); ++i) {
//       core_list += std::to_string(big_cores[i]);
//       if (i + 1 < big_cores.size()) core_list += ",";
//     }
//     tools::logger()->warn("[YOLO26] Big core ids: [{}]", core_list);
//   }
//   if (bind_current_thread_to_cores(big_cores)) {
//     tools::logger()->info("[YOLO26] Bound to big cores: {}", big_cores.size());
//   } else {
//     tools::logger()->warn("[YOLO26] Big core binding unavailable, using default affinity");
//   }
//   // 同步模式禁用OpenCV线程池，避免与OpenVINO争用
//   if (!use_async_inference_) {
//     cv::setNumThreads(1);
//   }

//   int x = 0, y = 0, width = 0, height = 0;
//   x = yaml["roi"]["x"].as<int>();
//   y = yaml["roi"]["y"].as<int>();
//   width = yaml["roi"]["width"].as<int>();
//   height = yaml["roi"]["height"].as<int>();
//   use_roi_ = yaml["use_roi"].as<bool>();
//   roi_ = cv::Rect(x, y, width, height);
//   offset_ = cv::Point2f(x, y);

//   save_path_ = "imgs";
//   std::filesystem::create_directory(save_path_);
//   auto model = core_.read_model(model_path_);
//   ov::preprocess::PrePostProcessor ppp(model);
//   auto & input = ppp.input();

//   input.tensor()
//     .set_element_type(ov::element::u8)
//     .set_shape({1, 640, 640, 3})
//     .set_layout("NHWC")
//     .set_color_format(ov::preprocess::ColorFormat::BGR);

//   input.model().set_layout("NCHW");

//   input.preprocess()
//     .convert_element_type(ov::element::f32)
//     .convert_color(ov::preprocess::ColorFormat::RGB)
//     .scale(255.0);

//   // TODO: ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
//   model = ppp.build();

//   // 编译模型：同步/异步都使用“最快组合”，但允许yaml覆盖线程/流数量
//   const int default_threads = use_async_inference_
//     ? (big_cores.empty() ? 8 : std::min(8, static_cast<int>(big_cores.size())))
//     : (big_cores.empty() ? 4 : std::min(4, static_cast<int>(big_cores.size())));
//   const int infer_threads = yaml["infer_threads"]
//     ? std::max(1, yaml["infer_threads"].as<int>())
//     : default_threads;
//   const int streams = yaml["ov_streams"]
//     ? std::max(1, yaml["ov_streams"].as<int>())
//     : (use_async_inference_ ? 2 : 1);
//   tools::logger()->info("[YOLO26] OpenVINO threads={}, streams={}, cpu_cores={}", infer_threads, streams, cpu_cores);
//   if (use_async_inference_) {
//     compiled_model_ = core_.compile_model(
//       model, device_,
//       ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT),
//       ov::streams::num(streams),
//       ov::hint::num_requests(PIPELINE_DEPTH),
//       ov::inference_num_threads(infer_threads)
//     );
//     tools::logger()->info("[YOLO26] Async fastest combo: threads={}, streams=2, requests={} ",
//                           infer_threads, PIPELINE_DEPTH);
//   } else {
//     compiled_model_ = core_.compile_model(
//       model, device_,
//       ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
//       ov::streams::num(streams),
//       ov::hint::num_requests(1),
//       ov::inference_num_threads(infer_threads)
//     );
//     tools::logger()->info("[YOLO26] Sync fastest combo: threads={}, streams=1, requests=1", infer_threads);
//   }

//   // 优化点3&4: 根据模式创建InferRequest
//   // 同步模式：只需要1个InferRequest
//   // 异步模式：需要PIPELINE_DEPTH个InferRequest
//   int num_requests = use_async_inference_ ? PIPELINE_DEPTH : 1;

//   for (int i = 0; i < num_requests; i++) {
//     infer_requests_[i] = compiled_model_.create_infer_request();

//     // 优化点4: 预先创建input_image和绑定Tensor，避免每帧构造
//     frame_contexts_[i].input_image = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
//     frame_contexts_[i].input_tensor = ov::Tensor(
//       ov::element::u8,
//       {1, 640, 640, 3},
//       frame_contexts_[i].input_image.data
//     );
//     // 预先绑定到InferRequest
//     infer_requests_[i].set_input_tensor(frame_contexts_[i].input_tensor);
//   }

//   tools::logger()->info("[YOLO26] Created {} InferRequest(s)", num_requests);
//   tools::logger()->info("[YOLO26] Async inference: {}", use_async_inference_ ? "ENABLED (深度异步pipeline)" : "DISABLED (同步模式)");
//   if (use_async_inference_) {
//     tools::logger()->info("[YOLO26] Pipeline: depth={}, 预绑定Tensor, 并行后处理", PIPELINE_DEPTH);
//   }
// }

// std::list<Armor> YOLO26::detect(const cv::Mat & raw_img, int frame_count)
// {
//   auto t_total_start = std::chrono::high_resolution_clock::now();

//   if (raw_img.empty()) {
//     tools::logger()->warn("Empty img!, camera drop!");
//     return std::list<Armor>();
//   }

//   if (use_async_inference_) {
//     // ==================== 极致优化的异步Pipeline模式 ====================
//     // 优化后的Pipeline顺序（深度=6，充分overlap + 并行后处理）：
//     // 1. 获取上一帧的后处理结果（如果有pending）
//     // 2. 等待上一帧推理完成
//     // 3. 启动后处理线程（异步，立即返回）
//     // 4. 预处理当前帧（与后处理并行）
//     // 5. 启动当前帧推理（异步，立即返回）
//     // 效果：最大化CPU利用率，后处理与预处理完全并行

//     std::list<Armor> result;

//     // 异步pipeline计时（轻量）：统计 preprocess / infer(latency) / wait-block / postprocess (async)
//     // 说明：
//     // - infer_latency：从 start_async() 到推理完成的总耗时（包含设备侧执行+调度）
//     // - wait_block：当前线程在 wait() 上阻塞的时间（越大表示overlap越差/推理没跟上）
//     // - 为避免引入性能抖动，仅在 debug_ 且固定间隔打印，并维护滑动平均用于观察趋势
//     static std::array<std::chrono::high_resolution_clock::time_point, PIPELINE_DEPTH> infer_start_ts;
//     static bool infer_start_ts_inited = false;
//     if (!infer_start_ts_inited) {
//       infer_start_ts.fill(std::chrono::high_resolution_clock::time_point{});
//       infer_start_ts_inited = true;
//     }
//     static double ema_pre_ms = 0.0;
//     static double ema_infer_ms = 0.0;
//     static double ema_wait_ms = 0.0;
//     constexpr double kEmaAlpha = 0.1;
//     static int timing_log_counter = 0;

//     // ========== 阶段1: 获取pending的后处理结果 ==========
//     if (has_pending_postprocess_ && postprocess_future_.valid()) {
//       // 非阻塞检查是否ready
//       if (postprocess_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
//         result = postprocess_future_.get();
//         has_pending_postprocess_ = false;
//       }
//     }

//     // ========== 阶段2: 等待上一帧推理完成 ==========
//     if (!first_frame_) {
//       int prev_request_idx = (current_request_idx_ - 1 + PIPELINE_DEPTH) % PIPELINE_DEPTH;
//       FrameContext & prev_ctx = frame_contexts_[prev_request_idx];

//       auto t_wait_start = std::chrono::high_resolution_clock::now();
//       infer_requests_[prev_request_idx].wait();
//       auto t_wait_end = std::chrono::high_resolution_clock::now();
//       auto duration_wait_us = std::chrono::duration_cast<std::chrono::microseconds>(
//         t_wait_end - t_wait_start).count();

//       // infer latency：从start_async到wait结束
//       double infer_ms = 0.0;
//       if (infer_start_ts[prev_request_idx].time_since_epoch().count() != 0) {
//         infer_ms = std::chrono::duration_cast<std::chrono::microseconds>(
//           t_wait_end - infer_start_ts[prev_request_idx]).count() / 1000.0;
//       }

//       // 更新 EMA（滑动平均）
//       const double wait_ms = duration_wait_us / 1000.0;
//       if (ema_wait_ms == 0.0) {
//         ema_wait_ms = wait_ms;
//         ema_infer_ms = infer_ms;
//       } else {
//         ema_wait_ms = ema_wait_ms * (1.0 - kEmaAlpha) + wait_ms * kEmaAlpha;
//         ema_infer_ms = ema_infer_ms * (1.0 - kEmaAlpha) + infer_ms * kEmaAlpha;
//       }

//       // 优化点6: 减少debug日志，只在间隔帧打印
//       if (debug_ && (timing_log_counter++ % 30 == 0)) {
//         tools::logger()->info(
//           "[YOLO26 Timing][async] infer(latency): {:.3f} ms (ema {:.3f}), wait(block): {:.3f} ms (ema {:.3f})",
//           infer_ms, ema_infer_ms, wait_ms, ema_wait_ms);
//       }

//       // ========== 阶段3: 启动异步后处理（与预处理并行）==========
//       // 优化点5: 使用std::async并行后处理
//       has_pending_postprocess_ = true;
//       postprocess_future_ = std::async(std::launch::async, [this, prev_request_idx, prev_ctx]() {
//         auto output_tensor = infer_requests_[prev_request_idx].get_output_tensor();
//         auto output_shape = output_tensor.get_shape();
//         cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

//         return parse(prev_ctx.scale, prev_ctx.pad_x, prev_ctx.pad_y, output,
//                      prev_ctx.bgr_img, prev_ctx.tmp_img, prev_ctx.frame_count);
//       });
//     }

//     // ========== 阶段4: 预处理当前帧（与后处理并行）==========
//   auto t_pre_start = std::chrono::high_resolution_clock::now();
//     FrameContext & current_ctx = frame_contexts_[current_request_idx_];
//     current_ctx.frame_count = frame_count;
//     current_ctx.valid = true;

//     if (debug_) {
//       current_ctx.tmp_img = raw_img.clone();
//     }

//     cv::Mat bgr_img;
//     if (use_roi_) {
//       if (roi_.width == -1) roi_.width = raw_img.cols;
//       if (roi_.height == -1) roi_.height = raw_img.rows;
//       bgr_img = raw_img(roi_);
//     } else {
//       bgr_img = raw_img;
//     }
//     current_ctx.bgr_img = bgr_img;

//     int orig_w = bgr_img.cols;
//     int orig_h = bgr_img.rows;
//     float scale = std::min(640.0f / orig_w, 640.0f / orig_h);
//     int new_w = static_cast<int>(orig_w * scale);
//     int new_h = static_cast<int>(orig_h * scale);
//     int pad_x = (640 - new_w) / 2;
//     int pad_y = (640 - new_h) / 2;

//     current_ctx.scale = scale;
//     current_ctx.pad_x = pad_x;
//     current_ctx.pad_y = pad_y;

//     // 优化点4: 直接复用预绑定的input_image，无需重新创建
//     current_ctx.input_image.setTo(cv::Scalar(0, 0, 0));
//     cv::resize(bgr_img, current_ctx.input_image(cv::Rect(pad_x, pad_y, new_w, new_h)),
//                cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

//     auto t_pre_end = std::chrono::high_resolution_clock::now();
//     const double pre_ms = std::chrono::duration_cast<std::chrono::microseconds>(t_pre_end - t_pre_start).count() / 1000.0;
//     if (ema_pre_ms == 0.0) {
//       ema_pre_ms = pre_ms;
//     } else {
//       ema_pre_ms = ema_pre_ms * (1.0 - kEmaAlpha) + pre_ms * kEmaAlpha;
//     }

//     // ========== 阶段5: 启动当前帧异步推理（不等待！）==========
//     // 优化点4: 无需每帧构造Tensor，直接使用预绑定的tensor
//     infer_start_ts[current_request_idx_] = std::chrono::high_resolution_clock::now();
//     infer_requests_[current_request_idx_].start_async();

//     if (debug_ && (timing_log_counter % 30 == 0)) {
//       tools::logger()->info("[YOLO26 Timing][async] preprocess: {:.3f} ms (ema {:.3f})", pre_ms, ema_pre_ms);
//     }

//     current_request_idx_ = (current_request_idx_ + 1) % PIPELINE_DEPTH;

//     // 优化点4&6: 简化首帧处理
//     if (first_frame_) {
//       tools::logger()->info("[YOLO26] 首帧启动 (pipeline深度={}，返回空)", PIPELINE_DEPTH);
//       first_frame_ = false;
//       return std::list<Armor>();
//     }

//     return result;  // 返回上一帧的结果（错位1帧）

//   } else {
//     // ==================== 同步模式 ====================
//     // ========== 阶段1: 预处理 ==========
//     auto t_preprocess_start = std::chrono::high_resolution_clock::now();

//     cv::Mat bgr_img;
//     cv::Mat tmp_img = raw_img;
//     if (use_roi_) {
//       if (roi_.width == -1) roi_.width = raw_img.cols;
//       if (roi_.height == -1) roi_.height = raw_img.rows;
//       bgr_img = raw_img(roi_);
//     } else {
//       bgr_img = raw_img;
//     }

//     int orig_w = bgr_img.cols;
//     int orig_h = bgr_img.rows;

//     float scale = std::min(640.0f / orig_w, 640.0f / orig_h);
//     int new_w = static_cast<int>(orig_w * scale);
//     int new_h = static_cast<int>(orig_h * scale);
//     int pad_x = (640 - new_w) / 2;
//     int pad_y = (640 - new_h) / 2;

//     // 优化：复用预绑定的input_image，避免每帧分配
//     frame_contexts_[0].input_image.setTo(cv::Scalar(0, 0, 0));
//     cv::resize(bgr_img, frame_contexts_[0].input_image(cv::Rect(pad_x, pad_y, new_w, new_h)),
//                cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

//     auto t_preprocess_end = std::chrono::high_resolution_clock::now();
//     auto duration_preprocess = std::chrono::duration_cast<std::chrono::microseconds>(t_preprocess_end - t_preprocess_start).count();
//     if (debug_) {
//       tools::logger()->info("[YOLO26 Timing]   预处理: {:.3f} ms", duration_preprocess / 1000.0);
//     }

//     // ========== 阶段2: 同步推理 ==========
//     auto t_infer_start = std::chrono::high_resolution_clock::now();
//     // 优化：使用预绑定的tensor，无需每帧set_input_tensor
//     infer_requests_[0].infer();  // 同步推理，阻塞等待
//     auto t_infer_end = std::chrono::high_resolution_clock::now();
//     auto duration_infer = std::chrono::duration_cast<std::chrono::microseconds>(t_infer_end - t_infer_start).count();
//     if (debug_) {
//       tools::logger()->info("[YOLO26 Timing]   同步推理: {:.3f} ms", duration_infer / 1000.0);
//     }

//     // ========== 阶段3: 后处理 ==========
//     auto t_postprocess_start = std::chrono::high_resolution_clock::now();
//     auto output_tensor = infer_requests_[0].get_output_tensor();
//     auto output_shape = output_tensor.get_shape();
//     cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

//     auto result = parse(scale, pad_x, pad_y, output, bgr_img, tmp_img, frame_count);

//     auto t_postprocess_end = std::chrono::high_resolution_clock::now();
//     auto duration_postprocess = std::chrono::duration_cast<std::chrono::microseconds>(t_postprocess_end - t_postprocess_start).count();
//     if (debug_) {
//       tools::logger()->info("[YOLO26 Timing]   后处理: {:.3f} ms", duration_postprocess / 1000.0);
//     }

//     // ========== 总结 ==========
//     auto t_total_end = std::chrono::high_resolution_clock::now();
//     auto duration_total = std::chrono::duration_cast<std::chrono::microseconds>(t_total_end - t_total_start).count();

//     if (debug_) {
//       tools::logger()->info("[YOLO26 Timing] ==================== 同步模式 ====================");
//       tools::logger()->info("[YOLO26 Timing] 预处理: {:.3f} ms ({:.1f}%)",
//                             duration_preprocess / 1000.0,
//                             100.0 * duration_preprocess / duration_total);
//       tools::logger()->info("[YOLO26 Timing] 推理:   {:.3f} ms ({:.1f}%)",
//                             duration_infer / 1000.0,
//                             100.0 * duration_infer / duration_total);
//       tools::logger()->info("[YOLO26 Timing] 后处理: {:.3f} ms ({:.1f}%)",
//                             duration_postprocess / 1000.0,
//                             100.0 * duration_postprocess / duration_total);
//       tools::logger()->info("[YOLO26 Timing] 总时间: {:.3f} ms", duration_total / 1000.0);
//       tools::logger()->info("[YOLO26 Timing] =======================================================");
//     }

//     return result;
//   }
// }
// //修改了这里
// std::list<Armor> YOLO26::parse(
//   double scale, int pad_x, int pad_y, cv::Mat & output, const cv::Mat & bgr_img,
//   const cv::Mat & tmp_img, int frame_count)
// {
//   // 调试：打印原始输出形状
//   if (debug_) {
//     tools::logger()->info("[YOLO26] Output shape: rows={}, cols={} (should be [N_detections, 18])",
//                           output.rows, output.cols);
//     tools::logger()->info("[YOLO26] bgr_img size: {}x{}, scale={}, pad_x={}, pad_y={}",
//                           bgr_img.cols, bgr_img.rows, scale, pad_x, pad_y);
//     tools::logger()->info("[YOLO26] use_roi_={}, offset_=({}, {})", use_roi_, offset_.x, offset_.y);
//   }

//   std::vector<int> ids;
//   std::vector<float> confidences;
//   std::vector<cv::Rect> boxes;
//   std::vector<std::vector<cv::Point2f>> armors_key_points;

//   int total_detections = 0;
//   int parsed_count = 0;
//   float max_conf = 0.0f;
//   float min_conf = 999.0f;

//   for (int r = 0; r < output.rows; r++) {
//     // 格式：[xyxy(4), confidence(1), class_id(1), keypoints(12)] = 18维
//     float x1_raw = output.at<float>(r, 0);
//     float y1_raw = output.at<float>(r, 1);
//     float x2_raw = output.at<float>(r, 2);
//     float y2_raw = output.at<float>(r, 3);
//     float conf = output.at<float>(r, 4);
//     int cls = static_cast<int>(output.at<float>(r, 5));

//     total_detections++;

//     // 统计置信度范围
//     max_conf = std::max(max_conf, conf);
//     min_conf = std::min(min_conf, conf);

//     // 调试：打印前5个检测的原始数据（不管是否通过阈值）
//     if (debug_ && total_detections <= 5) {
//       tools::logger()->info("[YOLO26] Raw detection {}: x1={:.1f}, y1={:.1f}, x2={:.1f}, y2={:.1f}, conf={:.6f}, cls={}, threshold={:.3f}",
//                             total_detections, x1_raw, y1_raw, x2_raw, y2_raw, conf, cls, score_threshold_);
//     }

//     // ---------- 1. 置信度过滤 ----------
//     if (conf < score_threshold_) {
//       continue;
//     }

//     parsed_count++;

//     // 调试：打印前几个检测的原始数据
//     if (debug_ && parsed_count <= 3) {
//       tools::logger()->info("[YOLO26] Detection {}: x1={:.1f}, y1={:.1f}, x2={:.1f}, y2={:.1f}, conf={:.3f}, cls={}",
//                             parsed_count, x1_raw, y1_raw, x2_raw, y2_raw, conf, cls);
//     }

//     // ---------- 2. bbox (xyxy) - 减去padding偏移并限制范围 ----------
//     int x1 = static_cast<int>((x1_raw - pad_x) / scale);
//     int y1 = static_cast<int>((y1_raw - pad_y) / scale);
//     int x2 = static_cast<int>((x2_raw - pad_x) / scale);
//     int y2 = static_cast<int>((y2_raw - pad_y) / scale);

//     // 限制坐标在图像范围内
//     x1 = std::clamp(x1, 0, bgr_img.cols);
//     x2 = std::clamp(x2, 0, bgr_img.cols);
//     y1 = std::clamp(y1, 0, bgr_img.rows);
//     y2 = std::clamp(y2, 0, bgr_img.rows);

//     int left   = x1;
//     int top    = y1;
//     int width  = std::max(0, x2 - x1);
//     int height = std::max(0, y2 - y1);

//     // 调试：打印坐标转换
//     if (debug_ && parsed_count <= 3) {
//       tools::logger()->info("[YOLO26] After transform: left={}, top={}, width={}, height={}",
//                             left, top, width, height);
//     }

//     // ---------- 3. keypoints (4×3 格式：x, y, visibility) ----------
//     std::vector<cv::Point2f> armor_key_points;
//     for (int i = 0; i < 4; i++) {
//       float kx_raw = output.at<float>(r, 6 + i * 3 + 0);
//       float ky_raw = output.at<float>(r, 6 + i * 3 + 1);
//       // float visibility = output.at<float>(r, 6 + i * 3 + 2);  // 忽略

//       float kx = (kx_raw - pad_x) / scale;
//       float ky = (ky_raw - pad_y) / scale;
//       armor_key_points.emplace_back(kx, ky);
//     }

//     ids.emplace_back(cls);
//     confidences.emplace_back(conf);
//     boxes.emplace_back(left, top, width, height);
//     armors_key_points.emplace_back(armor_key_points);
//   }

//   // ---------- 5. NMS (非极大值抑制) ----------
//   std::vector<int> indices;
//   cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

//   if (debug_) {
//     tools::logger()->info("[YOLO26] Total detections: {}, Max conf: {:.6f}, Min conf: {:.6f}",
//                           total_detections, max_conf, min_conf);
//     tools::logger()->info("[YOLO26] Passed threshold ({}): {} detections, After NMS: {} detections",
//                           score_threshold_, boxes.size(), indices.size());
//   }

//   // ---------- 6. 生成 Armor ----------
//   std::list<Armor> armors;
//   for (int i : indices) {
//     sort_keypoints(armors_key_points[i]);

//     if (debug_) {
//       tools::logger()->info("[YOLO26] Creating armor {}: id={}, conf={:.3f}, box=({},{},{}x{})",
//                             i, ids[i], confidences[i],
//                             boxes[i].x, boxes[i].y, boxes[i].width, boxes[i].height);
//     }

//     if (use_roi_) {
//       if (debug_) {
//         tools::logger()->info("[YOLO26] ROI mode: adding offset ({}, {})", offset_.x, offset_.y);
//       }
//       armors.emplace_back(
//         ids[i],
//         confidences[i],
//         boxes[i],
//         armors_key_points[i],
//         offset_,
//         YOLOVersion::YOLO26
//       );
//     } else {
//       armors.emplace_back(
//         ids[i],
//         confidences[i],
//         boxes[i],
//         armors_key_points[i],
//         YOLOVersion::YOLO26
//       );
//     }
//   }

//   if (debug_) {
//     tools::logger()->info("[YOLO26] Created {} armors", armors.size());
//   }

//   // ---------- 7. 过滤 ----------
//   int filtered_count = 0;
//   for (auto it = armors.begin(); it != armors.end();) {
//     bool name_ok = check_name(*it);
//     bool type_ok = check_type(*it);

//     if (!name_ok || !type_ok) {
//       if (debug_) {
//         tools::logger()->info("[YOLO26] Filtered out armor: name={}, type={}, name_ok={}, type_ok={}",
//                               static_cast<int>(it->name), static_cast<int>(it->type), name_ok, type_ok);
//       }
//       filtered_count++;
//       it = armors.erase(it);
//       continue;
//     }
//     // 注意：此时armor.center已经是全局坐标（相对于raw_img/tmp_img），所以归一化也要用tmp_img的尺寸
//     it->center_norm = get_center_norm(tmp_img, it->center);
//     ++it;
//   }

//   if (debug_) {
//     tools::logger()->info("[YOLO26] After filtering: {} armors remain, {} filtered out",
//                           armors.size(), filtered_count);
//   }

//   // 绘制检测结果（仅在debug模式，且不在异步pipeline中调用cv::imshow）
//   if (debug_ && !tmp_img.empty()) {
//     if (debug_) {
//       tools::logger()->info("[YOLO26] Calling draw_detections with {} armors", armors.size());
//     }
//     // 注意：异步模式下tmp_img可能为空（非debug模式不clone），此时不绘制
//     // 绘制函数内部不再调用cv::imshow，由外部控制
//     draw_detections(tmp_img, armors, frame_count);
//   }

//   return armors;
// }

// bool YOLO26::check_name(const Armor & armor) const
// {
//   auto name_ok = armor.name != ArmorName::not_armor;
//   auto confidence_ok = armor.confidence > min_confidence_;

//   // 保存不确定的图案，用于神经网络的迭代
//   // if (name_ok && !confidence_ok) save(armor);

//   return name_ok && confidence_ok;
// }

// bool YOLO26::check_type(const Armor & armor) const
// {
//   auto name_ok = (armor.type == ArmorType::small)
//                    ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
//                    : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
//                       armor.name != ArmorName::outpost);

//   // 保存异常的图案，用于神经网络的迭代
//   // if (!name_ok) save(armor);

//   return name_ok;
// }

// cv::Point2f YOLO26::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
// {
//   auto h = bgr_img.rows;
//   auto w = bgr_img.cols;
//   return {center.x / w, center.y / h};
// }

// void YOLO26::sort_keypoints(std::vector<cv::Point2f> & keypoints)
// {
//   if (keypoints.size() != 4) {
//     std::cout << "beyond 4!!" << std::endl;
//     return;
//   }

//   std::sort(keypoints.begin(), keypoints.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
//     return a.y < b.y;
//   });

//   std::vector<cv::Point2f> top_points = {keypoints[0], keypoints[1]};
//   std::vector<cv::Point2f> bottom_points = {keypoints[2], keypoints[3]};

//   std::sort(top_points.begin(), top_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
//     return a.x < b.x;
//   });

//   std::sort(
//     bottom_points.begin(), bottom_points.end(),
//     [](const cv::Point2f & a, const cv::Point2f & b) { return a.x < b.x; });

//   keypoints[0] = top_points[0];     // top-left
//   keypoints[1] = top_points[1];     // top-right
//   keypoints[2] = bottom_points[1];  // bottom-right
//   keypoints[3] = bottom_points[0];  // bottom-left
// }

// void YOLO26::draw_detections(
//   const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
// {
//   if (debug_) {
//     tools::logger()->info("[YOLO26] draw_detections called: img size={}x{}, armors count={}",
//                           img.cols, img.rows, armors.size());
//   }

//   // 优化：只在需要显示时才clone，否则直接在原图绘制（或跳过）
//   // 生产环境建议关闭debug以完全避免此开销
//   cv::Mat detection = img.clone();
//   tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

//   int armor_idx = 0;
//   for (const auto & armor : armors) {
//     if (debug_) {
//       tools::logger()->info("[YOLO26] Drawing armor {}: points[0]=({:.1f},{:.1f}), points[1]=({:.1f},{:.1f}), points[2]=({:.1f},{:.1f}), points[3]=({:.1f},{:.1f})",
//                             armor_idx,
//                             armor.points[0].x, armor.points[0].y,
//                             armor.points[1].x, armor.points[1].y,
//                             armor.points[2].x, armor.points[2].y,
//                             armor.points[3].x, armor.points[3].y);
//     }

//     auto info = fmt::format(
//       "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
//       ARMOR_TYPES[armor.type]);
//     tools::draw_points(detection, armor.points, {0, 255, 0});
//     tools::draw_text(detection, info, armor.center, {0, 255, 0});
//     armor_idx++;
//   }

//   if (use_roi_) {
//     cv::Scalar green(0, 255, 0);
//     cv::rectangle(detection, roi_, green, 2);
//   }
//   cv::resize(detection, detection, {}, 0.5, 0.5);  // 显示时缩小图片尺寸

//   // 注意：不在这里调用cv::imshow！
//   // 异步pipeline中调用cv::imshow会导致问题，应该由外部主线程控制显示
//   // 如果需要显示，可以在外部调用时单独处理
//   if (debug_) {
//     tools::logger()->info("[YOLO26] Detection image prepared ({}x{}), cv::imshow disabled in async mode",
//                           detection.cols, detection.rows);
//   }

//   // 可选：保存图像用于调试（同步模式）
//   // if (!use_async_inference_ && debug_) {
//   //   cv::imshow("detection", detection);
//   // }
// }

// // void YOLO26::save(const Armor & armor) const
// // {
// //   auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
// //   auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
// //   cv::imwrite(img_path, tmp_img_);
// // }

// std::list<Armor> YOLO26::postprocess(
//   double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
// {
//   // postprocess 假设没有 letterbox padding（外部调用场景）
//   // 使用bgr_img作为tmp_img（因为外部调用没有额外的原始图像）
//   return parse(scale, 0, 0, output, bgr_img, bgr_img, frame_count);
// }

// }  // namespace auto_aim