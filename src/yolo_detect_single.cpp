#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>

#include "tasks/auto_aim/yolos/yolo11.hpp"
#include "tools/img_tools.hpp"
#include "tasks/auto_aim/armor.hpp"

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <config_path> <input_image> <output_image>" << std::endl;
        return 1;
    }

    std::string config_path = argv[1];
    std::string input_path = argv[2];
    std::string output_path = argv[3];

    try {
        // 读取输入图片
        cv::Mat img = cv::imread(input_path);
        if (img.empty()) {
            std::cerr << "Error: Cannot read image from " << input_path << std::endl;
            return 1;
        }
        std::cout << "Image loaded: " << img.cols << "x" << img.rows << std::endl;

        // 初始化YOLO11检测器
        std::cout << "Initializing YOLO11 detector..." << std::endl;
        auto_aim::YOLO11 detector(config_path, false);  // debug=false

        // 执行检测
        std::cout << "Running detection..." << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        auto armors = detector.detect(img, 0);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        std::cout << "Detection completed in " << duration.count() << "ms" << std::endl;
        std::cout << "Found " << armors.size() << " armors" << std::endl;

        // 绘制检测结果
        cv::Mat result = img.clone();
        for (const auto& armor : armors) {
            // 绘制关键点连线
            if (armor.points.size() >= 4) {
                for (size_t i = 0; i < 4; i++) {
                    cv::line(result, armor.points[i], armor.points[(i + 1) % 4],
                            cv::Scalar(0, 255, 0), 2);
                }
            }

            // 绘制中心点
            cv::circle(result, armor.center, 5, cv::Scalar(0, 0, 255), -1);

            // 绘制文本信息
            std::string info = auto_aim::COLORS[armor.color] + " " +
                             auto_aim::ARMOR_NAMES[armor.name] + " " +
                             cv::format("%.2f", armor.confidence);

            int baseline = 0;
            cv::Size text_size = cv::getTextSize(info, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
            cv::Point text_org(armor.center.x - text_size.width / 2,
                             armor.center.y - 10);

            // 绘制文本背景
            cv::rectangle(result,
                        text_org + cv::Point(0, baseline),
                        text_org + cv::Point(text_size.width, -text_size.height),
                        cv::Scalar(0, 0, 0), -1);

            // 绘制文本
            cv::putText(result, info, text_org, cv::FONT_HERSHEY_SIMPLEX,
                       0.6, cv::Scalar(0, 255, 0), 2);
        }

        // 保存结果
        cv::imwrite(output_path, result);
        std::cout << "Result saved to " << output_path << std::endl;

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
