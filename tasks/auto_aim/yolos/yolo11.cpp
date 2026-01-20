#include "yolo11.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{

YOLO11::YOLO11(const std::string & config_path, bool debug) 
: debug_(debug), detector_(config_path, false)  
{
    
    auto yaml = YAML::LoadFile(config_path);

    
    model_path_ = yaml["yolo11_model_path"].as<std::string>();
    device_ = yaml["device"].as<std::string>();
    binary_threshold_ = yaml["threshold"].as<double>();
    min_confidence_ = yaml["min_confidence"].as<double>();
    use_async_inference_ = yaml["use_async_inference"].as<bool>();

    
    int x = 0, y = 0, width = 0, height = 0;
    x = yaml["roi"]["x"].as<int>();
    y = yaml["roi"]["y"].as<int>();
    width = yaml["roi"]["width"].as<int>();
    height = yaml["roi"]["height"].as<int>();
    use_roi_ = yaml["use_roi"].as<bool>();
    roi_ = cv::Rect(x, y, width, height);
    offset_ = cv::Point2f(x, y);           

    
    save_path_ = "imgs";
    std::filesystem::create_directory(save_path_);

    
    auto model = core_.read_model(model_path_);
    ov::preprocess::PrePostProcessor ppp(model);  
    auto & input = ppp.input();


    input.tensor()
        .set_element_type(ov::element::u8)
        .set_shape({1, 640, 640, 3})
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR);

    input.model().set_layout("NCHW");


    input.preprocess()
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale(255.0);                                    

    model = ppp.build();

    // 根据配置选择同步或异步推理模式
    if (use_async_inference_) {
        // 异步推理模式：适度并行，避免TBB任务爆炸
        compiled_model_ = core_.compile_model(
            model, device_,
            ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),  // 低延迟模式
            ov::streams::num(2),           // 只用2个推理流（减少并行度）
            ov::inference_num_threads(8),  // 限制为8线程（避免24线程的过度调度）
            ov::hint::num_requests(2)      // 2个请求（pipeline最小值）
        );

        // 创建2个异步推理请求，形成流水线（一个执行，一个准备）
        infer_request_current_ = compiled_model_.create_infer_request();
        infer_request_next_ = compiled_model_.create_infer_request();
        tools::logger()->info("[YOLO11] Async inference pipeline initialized (controlled parallelism)");
    } else {
        // 同步推理模式：低延迟优先
        compiled_model_ = core_.compile_model(
            model, device_,
            ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)  // 延迟模式，优化单帧速度
        );

        // 只创建一个推理请求用于同步推理
        infer_request_current_ = compiled_model_.create_infer_request();
        tools::logger()->info("[YOLO11] Sync inference mode initialized (low-latency)");
    }
}

std::list<Armor> YOLO11::detect(const cv::Mat & raw_img, int frame_count)
{
    if (raw_img.empty()) {
        tools::logger()->warn("Empty img!, camera drop!");
        return std::list<Armor>();
    }

    cv::Mat bgr_img;
    tmp_img_ = raw_img;

    // ROI 裁剪
    if (use_roi_) {
        int w = (roi_.width == -1) ? raw_img.cols : roi_.width;
        int h = (roi_.height == -1) ? raw_img.rows : roi_.height;
        cv::Rect roi(roi_.x, roi_.y, w, h);
        bgr_img = raw_img(roi);
    } else {
        bgr_img = raw_img;
    }

    int orig_w = bgr_img.cols;
    int orig_h = bgr_img.rows;

    // Letterbox resize 到 640x640
    float scale = std::min(640.0f / orig_w, 640.0f / orig_h);
    int new_w = static_cast<int>(orig_w * scale);
    int new_h = static_cast<int>(orig_h * scale);
    int pad_x = (640 - new_w) / 2;
    int pad_y = (640 - new_h) / 2;

    // 优化：使用static避免每帧重新分配内存
    static cv::Mat input(640, 640, CV_8UC3);
    input.setTo(cv::Scalar(0, 0, 0));  // 清零（比重新创建Mat快）

    // 使用INTER_LINEAR快速插值
    cv::resize(bgr_img, input(cv::Rect(pad_x, pad_y, new_w, new_h)),
               cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

    // 根据配置选择同步或异步推理
    if (use_async_inference_) {
        // ===== 异步推理模式（真正的Pipeline） =====
        if (first_frame_) {
            // === 第一帧：提交推理后立即返回空结果（初始化Pipeline） ===
            ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);
            infer_request_current_.set_input_tensor(input_tensor);
            infer_request_current_.start_async();  // 异步启动，不等待

            // 保存当前帧参数供下一帧使用
            prev_raw_img_ = raw_img;
            prev_scale_ = scale;
            prev_pad_x_ = pad_x;
            prev_pad_y_ = pad_y;
            prev_frame_count_ = frame_count;
            first_frame_ = false;

            if (debug_) {
                tools::logger()->info("[YOLO11] Async pipeline initialized, returning empty for frame 0");
            }

            return std::list<Armor>();  // 第一帧返回空，一帧延迟
        }
        else {
            // === Pipeline核心：重排操作顺序实现真正的并行 ===

            // 步骤1: 等待上一帧推理完成（可能已经完成，或者等待很短时间）
            infer_request_current_.wait();

            // 步骤2: 获取上一帧推理结果（此时 CPU 开始工作）
            auto output_tensor = infer_request_current_.get_output_tensor();
            auto output_shape = output_tensor.get_shape();
            cv::Mat output(static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]), CV_32F, output_tensor.data());

            // 步骤3: 提交当前帧推理（在后台开始推理，不阻塞）
            ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);
            infer_request_next_.set_input_tensor(input_tensor);
            infer_request_next_.start_async();  // 异步启动，立即返回

            // 步骤4: 处理上一帧结果（parse + NMS，CPU密集，此时GPU在推理当前帧）
            // 这里形成了真正的并行：CPU做后处理 || GPU做当前帧推理
            auto result = parse(prev_scale_, prev_pad_x_, prev_pad_y_, output, prev_raw_img_, prev_frame_count_);

            // 步骤5: 保存当前帧参数供下一帧使用
            prev_raw_img_ = raw_img;
            prev_scale_ = scale;
            prev_pad_x_ = pad_x;
            prev_pad_y_ = pad_y;
            prev_frame_count_ = frame_count;

            // 步骤6: 交换请求对象（下次循环时 current 指向刚提交的推理）
            std::swap(infer_request_current_, infer_request_next_);

            return result;  // 返回上一帧的检测结果（一帧延迟）
        }
    } else {
        // ===== 同步推理模式 =====
        ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);
        infer_request_current_.set_input_tensor(input_tensor);
        infer_request_current_.infer();  // 同步推理

        auto output_tensor = infer_request_current_.get_output_tensor();
        auto output_shape = output_tensor.get_shape();
        cv::Mat output(static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]), CV_32F, output_tensor.data());

        return parse(scale, pad_x, pad_y, output, raw_img, frame_count);
    }
}

std::list<Armor> YOLO11::parse(
    float scale, int pad_x, int pad_y, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
    
    if (debug_) {
        tools::logger()->info("Parse input: rows={}, cols={}", output.rows, output.cols);
    }

    std::vector<int> ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point2f>> armors_key_points;
    std::vector<std::vector<float>> keypoints_visibility;

    int detections_before_threshold = 0;
    for (int r = 0; r < output.rows; r++) {
        // 新格式：[xyxy(4), confidence(1), class_id(1), keypoints(12)] = 18维
        auto xyxy = output.row(r).colRange(0, 4);           // xyxy
        float confidence = output.row(r).at<float>(4);      // 置信度
        int class_id = static_cast<int>(output.row(r).at<float>(5)); // 类别ID
        auto one_key_points = output.row(r).colRange(6, 6 + 12); // 关键点从第7个位置开始（索引6）

        detections_before_threshold++;

        // 检查置信度阈值
        if (confidence < score_threshold_) continue;

        if (debug_ && ids.size() < 5) {  // 只打印前5个检测
            tools::logger()->info("Detection: confidence={:.3f}, class_id={}", confidence, class_id);
        }

        
        int x1 = static_cast<int>((xyxy.at<float>(0) - pad_x) / scale);
        int y1 = static_cast<int>((xyxy.at<float>(1) - pad_y) / scale);
        int x2 = static_cast<int>((xyxy.at<float>(2) - pad_x) / scale);
        int y2 = static_cast<int>((xyxy.at<float>(3) - pad_y) / scale);
        x1 = std::clamp(x1, 0, bgr_img.cols);
        x2 = std::clamp(x2, 0, bgr_img.cols);
        y1 = std::clamp(y1, 0, bgr_img.rows);
        y2 = std::clamp(y2, 0, bgr_img.rows);
        int width  = std::max(0, x2 - x1);
        int height = std::max(0, y2 - y1);

        std::vector<cv::Point2f> armor_key_points;
        std::vector<float> visibilities;
        for (int i = 0; i < 4; i++) {
            float kx = (one_key_points.at<float>(0, i*3 + 0) - pad_x) / scale;
            float ky = (one_key_points.at<float>(0, i*3 + 1) - pad_y) / scale;
            float v  = one_key_points.at<float>(0, i*3 + 2); 
            armor_key_points.emplace_back(kx, ky);
            visibilities.push_back(v);
        }

        ids.emplace_back(class_id);
        confidences.emplace_back(confidence);
        boxes.emplace_back(x1, y1, width, height);
        armors_key_points.emplace_back(armor_key_points);
        keypoints_visibility.emplace_back(visibilities);
    }

    if (debug_) {
        tools::logger()->info("Total detections: {}, After threshold: {}", detections_before_threshold, boxes.size());
    }

    // NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

    if (debug_) {
        tools::logger()->info("After NMS: {} detections", indices.size());
    }

    std::list<Armor> armors;
    for (auto i : indices) {
        sort_keypoints(armors_key_points[i]); 
        if (use_roi_) {
            armors.emplace_back(ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
        } else {
            armors.emplace_back(ids[i], confidences[i], boxes[i], armors_key_points[i]);
        }
    }

    if (debug_) {
        tools::logger()->info("Before filtering: {} armors", armors.size());
    }

    
    int filtered_count = 0;
    for (auto it = armors.begin(); it != armors.end();) {
        bool name_ok = check_name(*it);
        bool type_ok = check_type(*it);
        if (!name_ok || !type_ok) {
            if (debug_ && filtered_count < 5) {
                tools::logger()->info("Filtered: name_ok={}, type_ok={}, name={}, confidence={:.3f}",
                    name_ok, type_ok, static_cast<int>(it->name), it->confidence);
            }
            filtered_count++;
            it = armors.erase(it);
            continue;
        }
        it->center_norm = get_center_norm(bgr_img, it->center);
        ++it;
    }

    if (debug_) {
        tools::logger()->info("After filtering: {} armors (filtered out: {})", armors.size(), filtered_count);
    }

    if (debug_) draw_detections(bgr_img, armors, frame_count);
    return armors;
}



bool YOLO11::check_name(const Armor & armor) const {
    auto name_ok = armor.name != ArmorName::not_armor;
    auto confidence_ok = armor.confidence > min_confidence_;
    return name_ok && confidence_ok;
}


bool YOLO11::check_type(const Armor & armor) const {
    auto name_ok = (armor.type == ArmorType::small)
        ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
        : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
           armor.name != ArmorName::outpost);
    return name_ok;
}


cv::Point2f YOLO11::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const {
    auto h = bgr_img.rows;
    auto w = bgr_img.cols;
    return {center.x / w, center.y / h};
}


void YOLO11::sort_keypoints(std::vector<cv::Point2f> & keypoints) {
    if (keypoints.size() != 4) {
        std::cout << "beyond 4!!" << std::endl;
        return;
    }

    
    std::sort(keypoints.begin(), keypoints.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
        return a.y < b.y;
    });

    std::vector<cv::Point2f> top_points = {keypoints[0], keypoints[1]};
    std::vector<cv::Point2f> bottom_points = {keypoints[2], keypoints[3]};

    
    std::sort(top_points.begin(), top_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) { return a.x < b.x; });
    std::sort(bottom_points.begin(), bottom_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) { return a.x < b.x; });

    keypoints[0] = top_points[0];     // top-left
    keypoints[1] = top_points[1];     // top-right
    keypoints[2] = bottom_points[1];  // bottom-right
    keypoints[3] = bottom_points[0];  // bottom-left
}


void YOLO11::draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const {
    auto detection = img.clone();
    tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
    for (const auto & armor : armors) {
        auto info = fmt::format("{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name], ARMOR_TYPES[armor.type]);
        tools::draw_points(detection, armor.points, {0, 255, 0});
        tools::draw_text(detection, info, armor.center, {0, 255, 0});
    }
    if (use_roi_) {
        cv::rectangle(detection, roi_, {0, 255, 0}, 2); 
    }
    cv::resize(detection, detection, {}, 0.5, 0.5); 
    cv::imshow("detection", detection);
}


void YOLO11::save(const Armor & armor) const {
    auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
    auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
    cv::imwrite(img_path, tmp_img_);
}


std::list<Armor> YOLO11::postprocess(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) {
    // 如果直接调用postprocess，假设没有padding
    return parse(scale, 0, 0, output, bgr_img, frame_count);
}

}  // namespace auto_aim
