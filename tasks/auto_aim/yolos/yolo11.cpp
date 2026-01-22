#include "yolo11.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{

YOLO11::YOLO11(const std::string & config_path, bool debug) 
: debug_(debug), detector_(config_path, false)  // ��ʼ�� debug ��־�� detector ����
{
    // ��ȡ YAML �����ļ�
    auto yaml = YAML::LoadFile(config_path);

    // �������л�ȡģ��·�����豸���͡���ֵ
    model_path_ = yaml["yolo11_model_path"].as<std::string>();
    device_ = yaml["device"].as<std::string>();
    binary_threshold_ = yaml["threshold"].as<double>();
    min_confidence_ = yaml["min_confidence"].as<double>();

    // ��ʼ�� ROI ����
    int x = 0, y = 0, width = 0, height = 0;
    x = yaml["roi"]["x"].as<int>();
    y = yaml["roi"]["y"].as<int>();
    width = yaml["roi"]["width"].as<int>();
    height = yaml["roi"]["height"].as<int>();
    use_roi_ = yaml["use_roi"].as<bool>();  // �Ƿ����� ROI
    roi_ = cv::Rect(x, y, width, height);   // OpenCV ����
    offset_ = cv::Point2f(x, y);           // ROI ƫ�ƣ��������껹ԭ

    // ������������Ŀ¼
    save_path_ = "imgs";
    std::filesystem::create_directory(save_path_);

    // ��ȡģ��
    auto model = core_.read_model(model_path_);
    ov::preprocess::PrePostProcessor ppp(model);  // ����Ԥ������
    auto & input = ppp.input();

    // �������� tensor
    input.tensor()
        .set_element_type(ov::element::u8)        // �������� uint8
        .set_shape({1, 640, 640, 3})              // ����ߴ� NHWC
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR); // ԭʼͼ��Ϊ BGR

    input.model().set_layout("NCHW");  // ģ���ڲ����� layout Ϊ NCHW

    // ��������Ԥ��������һ�� + RGB ת��
    input.preprocess()
        .convert_element_type(ov::element::f32)           // תΪ float32
        .convert_color(ov::preprocess::ColorFormat::RGB)  // BGR -> RGB
        .scale(255.0);                                    // ���� 255 ��һ��

    // �����������ģ��
    model = ppp.build();

    // ����ģ�͵�ָ���豸����ָ���ӳ��Ż�ģʽ
    compiled_model_ = core_.compile_model(
        model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

std::list<Armor> YOLO11::detect(const cv::Mat & raw_img, int frame_count)
{
    if (raw_img.empty()) {
        tools::logger()->warn("Empty img!, camera drop!");
        return std::list<Armor>();
    }

    cv::Mat bgr_img;
    tmp_img_ = raw_img; // ����ԭͼ���ڵ���/����

    // �ü� ROI
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

    // --- letterbox ���ŵ� 640x640 ---
    float scale = std::min(640.0f / orig_w, 640.0f / orig_h);
    int new_w = static_cast<int>(orig_w * scale);
    int new_h = static_cast<int>(orig_h * scale);
    int pad_x = (640 - new_w) / 2;
    int pad_y = (640 - new_h) / 2;

    cv::Mat input(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::resize(bgr_img, input(cv::Rect(pad_x, pad_y, new_w, new_h)), cv::Size(new_w, new_h));

    // OpenVINO Tensor
    ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

    // ����
    auto infer_request = compiled_model_.create_infer_request();
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();

    auto output_tensor = infer_request.get_output_tensor();
    auto output_shape = output_tensor.get_shape(); // e.g., {1, N, 32}
    cv::Mat output(static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]), CV_32F, output_tensor.data());

    // 调试信息：打印输出shape
    if (debug_) {
        tools::logger()->info("Model output shape: [{}, {}, {}]", output_shape[0], output_shape[1], output_shape[2]);
        tools::logger()->info("cv::Mat shape: rows={}, cols={}", output.rows, output.cols);
    }

    // �������
    return parse(scale, pad_x, pad_y, output, raw_img, frame_count);
}

std::list<Armor> YOLO11::parse(
    float scale, int pad_x, int pad_y, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
    // 注意：YOLO11 nms=true 时输出已经是 (300, 18) 格式，不需要转置
    // cv::transpose(output, output);

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

        // xyxy -> ԭͼ����
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

        // �����ؼ���
        std::vector<cv::Point2f> armor_key_points;
        std::vector<float> visibilities;
        for (int i = 0; i < 4; i++) {
            float kx = (one_key_points.at<float>(0, i*3 + 0) - pad_x) / scale;
            float ky = (one_key_points.at<float>(0, i*3 + 1) - pad_y) / scale;
            float v  = one_key_points.at<float>(0, i*3 + 2); // �ɼ���
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
        sort_keypoints(armors_key_points[i]); // ����
        if (use_roi_) {
            armors.emplace_back(ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
        } else {
            armors.emplace_back(ids[i], confidences[i], boxes[i], armors_key_points[i]);
        }
    }

    if (debug_) {
        tools::logger()->info("Before filtering: {} armors", armors.size());
    }

    // ���˲��Ϸ�
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


// ����/���Ŷȼ��
bool YOLO11::check_name(const Armor & armor) const {
    auto name_ok = armor.name != ArmorName::not_armor;
    auto confidence_ok = armor.confidence > min_confidence_;
    return name_ok && confidence_ok;
}

// ���ͼ��
bool YOLO11::check_type(const Armor & armor) const {
    auto name_ok = (armor.type == ArmorType::small)
        ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
        : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
           armor.name != ArmorName::outpost);
    return name_ok;
}

// ��һ����������
cv::Point2f YOLO11::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const {
    auto h = bgr_img.rows;
    auto w = bgr_img.cols;
    return {center.x / w, center.y / h};
}

// �� 4 ���ؼ�������
void YOLO11::sort_keypoints(std::vector<cv::Point2f> & keypoints) {
    if (keypoints.size() != 4) {
        std::cout << "beyond 4!!" << std::endl;
        return;
    }

    // �� y ���������
    std::sort(keypoints.begin(), keypoints.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
        return a.y < b.y;
    });

    std::vector<cv::Point2f> top_points = {keypoints[0], keypoints[1]};
    std::vector<cv::Point2f> bottom_points = {keypoints[2], keypoints[3]};

    // ���¸��԰� x ����
    std::sort(top_points.begin(), top_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) { return a.x < b.x; });
    std::sort(bottom_points.begin(), bottom_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) { return a.x < b.x; });

    keypoints[0] = top_points[0];     // top-left
    keypoints[1] = top_points[1];     // top-right
    keypoints[2] = bottom_points[1];  // bottom-right
    keypoints[3] = bottom_points[0];  // bottom-left
}

// ���Ƽ����
void YOLO11::draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const {
    auto detection = img.clone();
    tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
    for (const auto & armor : armors) {
        auto info = fmt::format("{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name], ARMOR_TYPES[armor.type]);
        tools::draw_points(detection, armor.points, {0, 255, 0});
        tools::draw_text(detection, info, armor.center, {0, 255, 0});
    }
    if (use_roi_) {
        cv::rectangle(detection, roi_, {0, 255, 0}, 2); // ���� ROI
    }
    cv::resize(detection, detection, {}, 0.5, 0.5); // ��С��ʾ
    cv::imshow("detection", detection);
}

// ����ͼ��
void YOLO11::save(const Armor & armor) const {
    auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
    auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
    cv::imwrite(img_path, tmp_img_);
}

// postprocess ֱ�ӵ��� parse
std::list<Armor> YOLO11::postprocess(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) {
    // 如果直接调用postprocess，假设没有padding
    return parse(scale, 0, 0, output, bgr_img, frame_count);
}

}  // namespace auto_aim

