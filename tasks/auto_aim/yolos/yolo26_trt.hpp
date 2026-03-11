#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include <opencv2/opencv.hpp>

#include <chrono>
#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <mutex>
#include <queue>
#include <thread>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <climits>
#include <memory>
#include <cstring>

// 使用大恒相机时在编译时加 -DUSE_DAHENG，并链接 gxiapi、指定 io/driver 头路径
#ifdef USE_DAHENG
#include "driver/GxIAPI.h"
#endif

// ================== 配置 ==================
// TensorRT 使用 .engine 文件（可由 ONNX 用 trtexec 或代码生成）；若不存在则尝试从 ONNX 构建
static const std::string MODEL_ENGINE = "/home/nuc/Desktop/test_openvino/aim/repair_int8_openvino_model/repair.engine";
static const std::string MODEL_ONNX   = "/home/nuc/Desktop/test_openvino/aim/repair_int8_openvino_model/repair.onnx";  // 可选，engine 不存在时用
#ifndef USE_DAHENG
static const std::string VIDEO_PATH = "test.mp4";
#endif

static constexpr int INPUT_SIZE = 640;
static constexpr int NUM_CLASSES = 16;
static constexpr int NUM_KPTS = 4;

static constexpr float SCORE_THRESH = 0.3f;
static constexpr float NMS_THRESH   = 0.7f;

static constexpr int WARMUP = 10;
static constexpr bool VISUALIZE = true;
static constexpr int POSTPROCESS_DEBUG_FRAMES = 3;
static constexpr int PRINT_DET_EVERY_N_FRAMES = 1;
static constexpr int DAHENG_DEVICE_INDEX = 1;
static const std::string DAHENG_CAMERA_YAML_NAME = "daheng_camera.yaml";
static constexpr bool PRINT_CAMERA_CONFIG_ONLY = false;

// ================== TensorRT Logger ==================
class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cerr << "[Trt] " << msg << std::endl;
    }
};

// ================== Letterbox ==================
struct LetterBoxInfo {
    float ratio;
    int pad_x;
    int pad_y;
};

cv::Mat letterbox(
    const cv::Mat& img,
    LetterBoxInfo& info,
    int new_size = 640,
    const cv::Scalar& color = {114, 114, 114}
) {
    int w = img.cols;
    int h = img.rows;
    float r = std::min((float)new_size / w, (float)new_size / h);
    int new_w = int(w * r);
    int new_h = int(h * r);
    cv::Mat resized;
    cv::resize(img, resized, {new_w, new_h});
    cv::Mat padded(new_size, new_size, CV_8UC3, color);
    int pad_x = (new_size - new_w) / 2;
    int pad_y = (new_size - new_h) / 2;
    resized.copyTo(padded(cv::Rect(pad_x, pad_y, new_w, new_h)));
    info = {r, pad_x, pad_y};
    return padded;
}

// 将 cv::Mat (HWC, RGB, float32) 转为 NCHW 并写入 buffer（连续内存）
void mat_to_nchw_buffer(const cv::Mat& img, float* buffer) {
    int H = img.rows;
    int W = img.cols;
    int C = img.channels();
    std::vector<cv::Mat> channels(C);
    cv::split(img, channels);
    for (int c = 0; c < C; ++c)
        memcpy(buffer + c * H * W, channels[c].data, H * W * sizeof(float));
}

// ================== NMS (xyxy) ==================
std::vector<int> nms_xyxy(
    const std::vector<cv::Rect2f>& boxes,
    const std::vector<float>& scores,
    float iou_thresh
) {
    std::vector<int> indices(boxes.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
              [&](int a, int b) { return scores[a] > scores[b]; });
    std::vector<int> keep;
    while (!indices.empty()) {
        int i = indices[0];
        keep.push_back(i);
        std::vector<int> rest;
        for (size_t j = 1; j < indices.size(); j++) {
            int k = indices[j];
            float inter = (boxes[i] & boxes[k]).area();
            float uni = boxes[i].area() + boxes[k].area() - inter;
            if (inter / uni <= iou_thresh)
                rest.push_back(k);
        }
        indices.swap(rest);
    }
    return keep;
}

// ================== 后处理（与 OpenVINO 版一致，接口改为裸指针） ==================
struct Detection {
    cv::Rect2f box;
    int cls;
    float score;
    std::vector<cv::Point2f> kpts;
};

static constexpr int OUTPUT_STRIDE = 14;

// data: 输出指针，layout 每行 OUTPUT_STRIDE 个 float [x1,y1,x2,y2,conf,cls,kpt0_x,y,...,kpt3_x,y]
std::vector<Detection> postprocess_yolon26(
    const float* data,
    int num_rows,
    int stride,
    const LetterBoxInfo& info
) {
    if (stride != OUTPUT_STRIDE) return {};

    static int debug_printed = 0;
    const bool do_print = (POSTPROCESS_DEBUG_FRAMES != 0) &&
        (POSTPROCESS_DEBUG_FRAMES < 0 || debug_printed < POSTPROCESS_DEBUG_FRAMES);
    if (do_print) {
        if (POSTPROCESS_DEBUG_FRAMES > 0) debug_printed++;
        std::cout << "[postprocess_yolon26] rows=" << num_rows << " stride=" << stride << std::endl;
        float max_conf = 0.f;
        int best_row = 0;
        for (int i = 0; i < num_rows; i++) {
            float c = data[i * stride + 4];
            if (c > max_conf) { max_conf = c; best_row = i; }
        }
        const float* r = data + best_row * stride;
        std::cout << "[postprocess_yolon26] row " << best_row << " raw 14: ";
        for (int j = 0; j < OUTPUT_STRIDE; j++) std::cout << r[j] << " ";
        std::cout << std::endl;
    }

    std::vector<cv::Rect2f> boxes;
    std::vector<float> scores;
    std::vector<int> cls_ids;
    std::vector<std::vector<cv::Point2f>> keypoints;

    for (int i = 0; i < num_rows; i++) {
        const float* p = data + i * stride;
        float conf = p[4];
        if (conf < SCORE_THRESH) continue;
        int cls = static_cast<int>(p[5]);
        float x1 = (p[0] - info.pad_x) / info.ratio;
        float y1 = (p[1] - info.pad_y) / info.ratio;
        float x2 = (p[2] - info.pad_x) / info.ratio;
        float y2 = (p[3] - info.pad_y) / info.ratio;
        boxes.emplace_back(cv::Rect2f(cv::Point2f(x1, y1), cv::Point2f(x2, y2)));
        scores.emplace_back(conf);
        cls_ids.emplace_back(cls);
        std::vector<cv::Point2f> kp;
        for (int k = 0; k < NUM_KPTS; k++) {
            float kx = (p[6 + k * 2 + 0] - info.pad_x) / info.ratio;
            float ky = (p[6 + k * 2 + 1] - info.pad_y) / info.ratio;
            kp.emplace_back(kx, ky);
        }
        keypoints.push_back(kp);
    }

    std::vector<int> keep = nms_xyxy(boxes, scores, NMS_THRESH);
    std::vector<Detection> detections;
    for (int idx : keep)
        detections.push_back({ boxes[idx], cls_ids[idx], scores[idx], keypoints[idx] });
    return detections;
}

// ================== TensorRT 推理封装 ==================
struct TrtContext {
    TrtLogger logger;
    nvinfer1::IRuntime* runtime = nullptr;
    nvinfer1::ICudaEngine* engine = nullptr;
    nvinfer1::IExecutionContext* context = nullptr;
    cudaStream_t stream = nullptr;

    int input_index = 0;
    int output_index = 0;
    size_t input_size_bytes = 0;
    size_t output_size_bytes = 0;
    int output_rows = 0;
    int output_stride = OUTPUT_STRIDE;

    float* d_input = nullptr;
    float* d_output = nullptr;
    std::vector<void*> bindings;

    ~TrtContext() { release(); }

    bool loadEngine(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f.good()) return false;
        f.seekg(0, std::ios::end);
        size_t size = f.tellg();
        f.seekg(0, std::ios::beg);
        std::vector<char> buf(size);
        f.read(buf.data(), size);
        f.close();

        runtime = nvinfer1::createInferRuntime(logger);
        if (!runtime) return false;
        engine = runtime->deserializeCudaEngine(buf.data(), size);
        if (!engine) { release(); return false; }
        context = engine->createExecutionContext();
        if (!context) { release(); return false; }
        return true;
    }

    bool buildFromOnnx(const std::string& onnxPath) {
        auto builder = std::unique_ptr<nvinfer1::IBuilder>(
            nvinfer1::createInferBuilder(logger));
        if (!builder) return false;
        const auto flags = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(
            builder->createNetworkV2(flags));
        if (!network) return false;
        auto parser = std::unique_ptr<nvonnxparser::IParser>(
            nvonnxparser::createParser(*network, logger));
        if (!parser || !parser->parseFromFile(onnxPath.c_str(),
                static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)))
            return false;
        auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
        if (!config) return false;
        config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 64ULL << 20);
        auto serialized = std::unique_ptr<nvinfer1::IHostMemory>(
            builder->buildSerializedNetwork(*network, *config));
        if (!serialized) return false;
        runtime = nvinfer1::createInferRuntime(logger);
        if (!runtime) return false;
        engine = runtime->deserializeCudaEngine(serialized->data(), serialized->size());
        if (!engine) { release(); return false; }
        context = engine->createExecutionContext();
        if (!context) { release(); return false; }
        return true;
    }

    bool init() {
        if (cudaSetDevice(0) != cudaSuccess) {
            std::cerr << "cudaSetDevice failed\n";
            return false;
        }
        if (cudaStreamCreate(&stream) != cudaSuccess) {
            std::cerr << "cudaStreamCreate failed\n";
            return false;
        }
        if (!loadEngine(MODEL_ENGINE)) {
            std::cout << "Engine not found, trying build from ONNX: " << MODEL_ONNX << std::endl;
            std::ifstream f(MODEL_ONNX);
            if (!f.good()) {
                std::cerr << "Neither engine nor ONNX found. Place " << MODEL_ENGINE
                          << " or " << MODEL_ONNX << std::endl;
                return false;
            }
            f.close();
            if (!buildFromOnnx(MODEL_ONNX)) {
                std::cerr << "Build from ONNX failed\n";
                return false;
            }
        }
        const int nb = engine->getNbBindings();
        for (int i = 0; i < nb; i++) {
            if (engine->bindingIsInput(i))
                input_index = i;
            else
                output_index = i;
        }
        nvinfer1::Dims input_dims = context->getBindingDimensions(input_index);
        nvinfer1::Dims output_dims = context->getBindingDimensions(output_index);
        // input: NCHW -> N*C*H*W
        size_t n = 1;
        for (int j = 0; j < input_dims.nbDims; j++) n *= input_dims.d[j];
        input_size_bytes = n * sizeof(float);
        n = 1;
        for (int j = 0; j < output_dims.nbDims; j++) n *= (output_dims.d[j] > 0 ? output_dims.d[j] : 1);
        output_size_bytes = n * sizeof(float);
        // 假设 output shape [1, num_boxes, 14] 或 [num_boxes, 14]
        if (output_dims.nbDims >= 2 && output_dims.d[output_dims.nbDims - 1] > 0) {
            output_stride = output_dims.d[output_dims.nbDims - 1];
            output_rows = static_cast<int>(n / output_stride);
        } else {
            output_stride = OUTPUT_STRIDE;
            output_rows = static_cast<int>(n / output_stride);
        }
        std::cout << "Input size " << input_size_bytes << " bytes, output " << output_rows << " x " << output_stride << std::endl;

        if (cudaMalloc(&d_input, input_size_bytes) != cudaSuccess ||
            cudaMalloc(&d_output, output_size_bytes) != cudaSuccess) {
            std::cerr << "cudaMalloc failed\n";
            release();
            return false;
        }
        bindings.resize(nb);
        bindings[input_index] = d_input;
        bindings[output_index] = d_output;
        return true;
    }

    void release() {
        if (stream) { cudaStreamSynchronize(stream); cudaStreamDestroy(stream); stream = nullptr; }
        if (d_input) { cudaFree(d_input); d_input = nullptr; }
        if (d_output) { cudaFree(d_output); d_output = nullptr; }
        if (context) { context->destroy(); context = nullptr; }
        if (engine) { engine->destroy(); engine = nullptr; }
        if (runtime) { runtime->destroy(); runtime = nullptr; }
    }

    bool infer(const float* host_input, float* host_output) {
        if (cudaMemcpyAsync(d_input, host_input, input_size_bytes, cudaMemcpyHostToDevice, stream) != cudaSuccess)
            return false;
        if (!context->enqueueV2(bindings.data(), stream, nullptr)) return false;
        if (cudaMemcpyAsync(host_output, d_output, output_size_bytes, cudaMemcpyDeviceToHost, stream) != cudaSuccess)
            return false;
        if (cudaStreamSynchronize(stream) != cudaSuccess) return false;
        return true;
    }

    int getOutputRows() const { return output_rows; }
    int getOutputStride() const { return output_stride; }
};

// ================== 大恒相机（与 OpenVINO 版相同） ==================
#ifdef USE_DAHENG
struct DahengCameraParams {
    int width = 1920;
    int height = 1200;
    double exposure = 2000;
    bool auto_exposure = false;
    double gain = 24;
    bool auto_white_balance = true;
    double rgain = 0.0, bgain = 0.0, ggain = 0.0;
    int max_exp = 3000, min_exp = 200;
};

static bool file_exists(const std::string& path) {
    std::ifstream f(path);
    return f.good();
}

static bool load_daheng_params(const std::string& yaml_path, DahengCameraParams& out, bool verbose = false) {
    if (!file_exists(yaml_path)) {
        if (verbose) std::cerr << "  [yaml] file not found: " << yaml_path << std::endl;
        return false;
    }
    std::string path = yaml_path;
#if defined(__linux__) || defined(__APPLE__)
    char buf[PATH_MAX];
    if (realpath(yaml_path.c_str(), buf)) path = buf;
#endif
    try {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;
        cv::FileNode settings = fs["camera_settings"];
        if (settings.empty()) { fs.release(); return false; }
        settings["width"] >> out.width;
        settings["height"] >> out.height;
        settings["exposure"] >> out.exposure;
        if (!settings["auto_exposure"].empty()) { double v = 0; settings["auto_exposure"] >> v; out.auto_exposure = (v > 0.5); }
        settings["gain"] >> out.gain;
        if (!settings["auto_white_balance"].empty()) { double v = 1; settings["auto_white_balance"] >> v; out.auto_white_balance = (v > 0.5); }
        settings["rgain"] >> out.rgain;
        settings["bgain"] >> out.bgain;
        settings["ggain"] >> out.ggain;
        settings["max_exp"] >> out.max_exp;
        settings["min_exp"] >> out.min_exp;
        fs.release();
        return true;
    } catch (const cv::Exception&) {
        return false;
    }
}

static void apply_daheng_params(GX_DEV_HANDLE dev, const DahengCameraParams& p) {
    GXSetInt(dev, GX_INT_WIDTH, p.width);
    GXSetInt(dev, GX_INT_HEIGHT, p.height);
    if (p.auto_exposure) {
        GXSetEnum(dev, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
        GXSetFloat(dev, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, static_cast<double>(p.max_exp));
        GXSetFloat(dev, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, static_cast<double>(p.min_exp));
    } else {
        GXSetEnum(dev, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
        GXSetFloat(dev, GX_FLOAT_EXPOSURE_TIME, p.exposure);
    }
    GXSetFloat(dev, GX_FLOAT_GAIN, p.gain);
    const GX_PIXEL_FORMAT_ENTRY fmts[] = { GX_PIXEL_FORMAT_BAYER_RG8, GX_PIXEL_FORMAT_BAYER_GR8, GX_PIXEL_FORMAT_BAYER_GB8, GX_PIXEL_FORMAT_BAYER_BG8 };
    for (int i = 0; i < 4; ++i) { if (GXSetEnum(dev, GX_ENUM_PIXEL_FORMAT, fmts[i]) == GX_STATUS_SUCCESS) break; }
    if (p.auto_white_balance) {
        GXSetEnum(dev, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    } else {
        GXSetEnum(dev, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
        if (p.rgain > 0) { GXSetEnum(dev, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);   GXSetFloat(dev, GX_FLOAT_BALANCE_RATIO, p.rgain); }
        if (p.bgain > 0) { GXSetEnum(dev, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);  GXSetFloat(dev, GX_FLOAT_BALANCE_RATIO, p.bgain); }
        if (p.ggain > 0) { GXSetEnum(dev, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN); GXSetFloat(dev, GX_FLOAT_BALANCE_RATIO, p.ggain); }
    }
    GXSetEnum(dev, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    GXSetEnum(dev, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
}

struct DahengCapture {
    GX_DEV_HANDLE dev = nullptr;
    int width = 0, height = 0;
    int64_t payload_size = 0;
    std::vector<unsigned char> buffer;

    bool open(const char* exe_path = nullptr, bool config_only = false) {
        if (GXInitLib() != GX_STATUS_SUCCESS) { std::cerr << "Daheng: GXInitLib failed\n"; return false; }
        uint32_t device_num = 0;
        if (GXUpdateDeviceList(&device_num, 1000) != GX_STATUS_SUCCESS) {
            std::cerr << "Daheng: GXUpdateDeviceList failed\n"; GXCloseLib(); return false;
        }
        if (device_num == 0) { std::cerr << "Daheng: no device found\n"; GXCloseLib(); return false; }
        uint32_t idx = static_cast<uint32_t>(DAHENG_DEVICE_INDEX);
        if (GXOpenDeviceByIndex(idx, &dev) != GX_STATUS_SUCCESS && idx == 1u && device_num >= 1u)
            GXOpenDeviceByIndex(0, &dev);
        if (dev == nullptr) { std::cerr << "Daheng: GXOpenDeviceByIndex failed\n"; GXCloseLib(); return false; }

        DahengCameraParams params;
        std::string yaml_path = DAHENG_CAMERA_YAML_NAME;
        if (exe_path) {
            std::string exe(exe_path);
            size_t last = exe.find_last_of("/\\");
            if (last != std::string::npos) {
                std::string dir = exe.substr(0, last);
                if (load_daheng_params(dir + "/" + DAHENG_CAMERA_YAML_NAME, params)) yaml_path = dir + "/" + DAHENG_CAMERA_YAML_NAME;
                else if (load_daheng_params(dir + "/../" + DAHENG_CAMERA_YAML_NAME, params)) yaml_path = dir + "/../" + DAHENG_CAMERA_YAML_NAME;
            }
        }
        if (!load_daheng_params(yaml_path, params)) load_daheng_params(DAHENG_CAMERA_YAML_NAME, params);
        apply_daheng_params(dev, params);

        if (config_only) {
            GXCloseDevice(dev); dev = nullptr; GXCloseLib();
            return true;
        }
        int64_t w = 0, h = 0;
        GXGetInt(dev, GX_INT_WIDTH, &w);
        GXGetInt(dev, GX_INT_HEIGHT, &h);
        GXGetInt(dev, GX_INT_PAYLOAD_SIZE, &payload_size);
        width = static_cast<int>(w);
        height = static_cast<int>(h);
        buffer.resize(static_cast<size_t>(payload_size));
        if (GXStreamOn(dev) != GX_STATUS_SUCCESS) {
            GXCloseDevice(dev); GXCloseLib(); return false;
        }
        return true;
    }

    bool read(cv::Mat& frame) {
        GX_FRAME_DATA fd;
        fd.pImgBuf = buffer.data();
        if (GXGetImage(dev, &fd, 100) != GX_STATUS_SUCCESS || fd.nStatus != GX_FRAME_STATUS_SUCCESS)
            return false;
        cv::Mat bayer(height, width, CV_8UC1, fd.pImgBuf);
        cv::cvtColor(bayer, frame, cv::COLOR_BayerBG2BGR);
        return true;
    }

    void release() {
        if (dev) { GXStreamOff(dev); GXCloseDevice(dev); dev = nullptr; }
        GXCloseLib();
    }
    ~DahengCapture() { release(); }
};
#endif

// ================== main ==================
int main(int argc, char** argv) {
#ifdef USE_DAHENG
    if (PRINT_CAMERA_CONFIG_ONLY) {
        DahengCapture daheng;
        if (!daheng.open(argc > 0 ? argv[0] : nullptr, true)) return -1;
        return 0;
    }
#endif

    TrtContext trt;
    if (!trt.init()) {
        std::cerr << "TensorRT init failed.\n";
        return -1;
    }
    std::cout << "TensorRT model loaded OK." << std::endl;

    const int out_rows = trt.getOutputRows();
    const int out_stride = trt.getOutputStride();
    std::vector<float> host_input(trt.input_size_bytes / sizeof(float));
    std::vector<float> host_output(trt.output_size_bytes / sizeof(float));

    cv::Mat frame;
#ifdef USE_DAHENG
    DahengCapture daheng;
    if (!daheng.open(argc > 0 ? argv[0] : nullptr, false)) {
        std::cerr << "Failed to open Daheng camera\n";
        return -1;
    }
#else
    cv::VideoCapture cap(VIDEO_PATH);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open video: " << VIDEO_PATH << "\n";
        return -1;
    }
#endif

    std::vector<double> infer_times;
    int frame_id = 0;
    std::cout << "Real-time inference (TensorRT). Press 'q' or ESC to quit.\n";

    for (;;) {
#ifdef USE_DAHENG
        if (!daheng.read(frame)) continue;
#else
        if (!cap.read(frame)) break;
#endif
        frame_id++;

        LetterBoxInfo info;
        cv::Mat frame_rgb;
        cv::cvtColor(frame, frame_rgb, cv::COLOR_BGR2RGB);
        cv::Mat input = letterbox(frame_rgb, info);
        input.convertTo(input, CV_32F, 1.0 / 255.0);
        mat_to_nchw_buffer(input, host_input.data());

        auto t0 = std::chrono::steady_clock::now();
        if (!trt.infer(host_input.data(), host_output.data())) {
            std::cerr << "Infer failed at frame " << frame_id << std::endl;
            continue;
        }
        auto t1 = std::chrono::steady_clock::now();

        if (frame_id > WARMUP)
            infer_times.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());

        auto dets = postprocess_yolon26(host_output.data(), out_rows, out_stride, info);

        if (VISUALIZE) {
            if (PRINT_DET_EVERY_N_FRAMES > 0 && (frame_id % PRINT_DET_EVERY_N_FRAMES == 0 || frame_id <= 1)) {
                std::cout << "[frame " << frame_id << "] dets=" << dets.size();
                if (!infer_times.empty())
                    std::cout << " infer_ms=" << std::fixed << std::setprecision(2) << infer_times.back();
                std::cout << std::endl;
                for (size_t i = 0; i < dets.size(); i++) {
                    const auto& d = dets[i];
                    std::cout << "  [" << i << "] cls=" << d.cls << " conf=" << d.score
                              << " box=(" << (int)d.box.x << "," << (int)d.box.y << ","
                              << (int)(d.box.x + d.box.width) << "," << (int)(d.box.y + d.box.height) << ")"
                              << std::endl;
                }
            }
            for (auto& d : dets) {
                cv::rectangle(frame, d.box, {0, 255, 0}, 2);
                for (auto& p : d.kpts)
                    cv::circle(frame, p, 3, {0, 0, 255}, -1);
            }
            if (!infer_times.empty()) {
                double fps = 1000.0 / infer_times.back();
                cv::putText(frame, "FPS: " + std::to_string(static_cast<int>(fps + 0.5)),
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            }
            cv::imshow("det", frame);
        }

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;
    }

#ifdef USE_DAHENG
#else
    cap.release();
#endif
    cv::destroyAllWindows();

    if (!infer_times.empty()) {
        double avg = std::accumulate(infer_times.begin(), infer_times.end(), 0.0) / infer_times.size();
        std::cout << "Frames: " << frame_id << " | Avg infer: " << avg << " ms | FPS: " << (1000.0 / avg) << "\n";
    }
    return 0;
}
