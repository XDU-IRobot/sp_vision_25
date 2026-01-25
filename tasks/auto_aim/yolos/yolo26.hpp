// #ifndef AUTO_AIM__YOLO26_HPP
// #define AUTO_AIM__YOLO26_HPP

// #include <future>
// #include <list>
// #include <opencv2/opencv.hpp>
// #include <openvino/openvino.hpp>
// #include <string>
// #include <vector>

// #include "tasks/auto_aim/armor.hpp"
// #include "tasks/auto_aim/detector.hpp"
// #include "tasks/auto_aim/yolo.hpp"

// namespace auto_aim
// {

// // 帧上下文：绑定每个InferRequest的输入帧信息
// struct FrameContext {
//   double scale;
//   int pad_x;
//   int pad_y;
//   cv::Mat input_image;  // 640x640预处理后的图像（持久化，避免tensor生命周期问题）
//   ov::Tensor input_tensor;  // 预先绑定的Tensor，避免每帧构造
//   cv::Mat bgr_img;      // ROI裁剪后的图像（仅保留引用，避免clone）
//   cv::Mat tmp_img;      // 原始图像（仅在debug模式需要，避免clone）
//   int frame_count;
//   bool valid;           // 该context是否有效

//   FrameContext() : scale(1.0), pad_x(0), pad_y(0), frame_count(0), valid(false) {}
// };

// class YOLO26 : public YOLOBase
// {
// public:
//   YOLO26(const std::string & config_path, bool debug);

//   std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

//   std::list<Armor> postprocess(
//     double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

// private:
//   std::string device_, model_path_;
//   std::string save_path_, debug_path_;
//   bool debug_, use_roi_;
//   bool use_async_inference_;  // 异步推理开关

//   const int class_num_ = 16;  // 和yolo11相同，16个类别
//   const float nms_threshold_ = 0.3;
//   //这里可以没有nms
//   const float score_threshold_ = 0.2;  // yolo26模型置信度异常低，临时降到0.2用于测试
//   double min_confidence_, binary_threshold_;

//   ov::Core core_;
//   ov::CompiledModel compiled_model_;

//   // 优化点3: 扩展buffer到6个请求，减少等待阻塞
//   static constexpr int PIPELINE_DEPTH = 6;
//   ov::InferRequest infer_requests_[PIPELINE_DEPTH];  // 6个InferRequest用于深度pipeline
//   FrameContext frame_contexts_[PIPELINE_DEPTH];      // 对应的帧上下文
//   int current_request_idx_;                          // 当前使用的request索引 (0-5)
//   bool first_frame_;                                 // 是否首帧标志

//   // 优化点5: 线程池用于并行后处理
//   std::future<std::list<Armor>> postprocess_future_;
//   bool has_pending_postprocess_;

//   cv::Rect roi_;
//   cv::Point2f offset_;

//   Detector detector_;

//   bool check_name(const Armor & armor) const;
//   bool check_type(const Armor & armor) const;

//   cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

//   std::list<Armor> parse(double scale, int pad_x, int pad_y, cv::Mat & output,
//                          const cv::Mat & bgr_img, const cv::Mat & tmp_img, int frame_count);

//   void save(const Armor & armor) const;
//   void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
//   void sort_keypoints(std::vector<cv::Point2f> & keypoints);
// };

// }  // namespace auto_aim

// #endif  //AUTO_AIM__YOLO26_HPP