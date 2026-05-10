#include "debug_recorder.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <unordered_map>
#include <utility>

#ifdef TOOLS_HAS_ROS2_DEBUG_RECORDER
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#endif

#include "logger.hpp"

namespace tools
{

#ifdef TOOLS_HAS_ROS2_DEBUG_RECORDER

class DebugRecorder::Impl
{
public:
  explicit Impl(const Options & options)
  {
    auto context = rclcpp::contexts::get_global_default_context();
    if (!context || !context->is_valid()) {
      logger()->warn("[DebugRecorder] rclcpp is not initialized, debug recorder disabled.");
      return;
    }

    // 使用 SensorDataQoS 风格的 best_effort + keep_last(1)。
    // 这样 rosbag/Foxglove 跟不上时会丢旧调试数据，而不是让主循环积压。
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    node = std::make_shared<rclcpp::Node>(options.node_name);
    if (options.publish_image) {
      image_pub = node->create_publisher<sensor_msgs::msg::Image>(options.image_topic, qos);
    }
    if (options.publish_json) {
      json_pub = node->create_publisher<std_msgs::msg::String>(options.json_topic, qos);
    }

    logger()->info(
      "[DebugRecorder] Enabled: image_topic={}, json_topic={}, scalar_prefix={}",
      options.image_topic, options.json_topic, options.scalar_topic_prefix);
  }

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_pub;
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
    scalar_pubs;
};

namespace
{

std::string sanitize_token(const std::string & token)
{
  std::string out;
  out.reserve(token.size());

  for (auto c : token) {
    const auto ch = static_cast<unsigned char>(c);
    if (std::isalnum(ch) || c == '_') {
      out.push_back(c);
    } else {
      out.push_back('_');
    }
  }

  if (out.empty()) {
    out = "value";
  }

  // ROS2 name token 不适合以数字开头，数组索引用 i0/i1 更稳。
  if (std::isdigit(static_cast<unsigned char>(out.front()))) {
    out.insert(out.begin(), 'i');
  }

  return out;
}

std::string join_topic(const std::string & prefix, const std::string & path)
{
  if (path.empty()) {
    return prefix;
  }
  if (prefix.empty() || prefix == "/") {
    return "/" + path;
  }
  if (prefix.back() == '/') {
    return prefix + path;
  }
  return prefix + "/" + path;
}

void publish_scalar(
  DebugRecorder::Impl & impl,
  const std::string & topic,
  double value)
{
  auto it = impl.scalar_pubs.find(topic);
  if (it == impl.scalar_pubs.end()) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    it = impl.scalar_pubs.emplace(topic, impl.node->create_publisher<std_msgs::msg::Float64>(topic, qos))
           .first;
  }

  std_msgs::msg::Float64 msg;
  msg.data = value;
  it->second->publish(msg);
}

void publish_json_scalars(
  DebugRecorder::Impl & impl,
  const nlohmann::json & json,
  const std::string & prefix,
  const std::string & path)
{
  if (json.is_number()) {
    publish_scalar(impl, join_topic(prefix, path), json.get<double>());
    return;
  }

  if (json.is_boolean()) {
    publish_scalar(impl, join_topic(prefix, path), json.get<bool>() ? 1.0 : 0.0);
    return;
  }

  if (json.is_object()) {
    for (const auto & item : json.items()) {
      const auto token = sanitize_token(item.key());
      const auto child_path = path.empty() ? token : path + "/" + token;
      publish_json_scalars(impl, item.value(), prefix, child_path);
    }
    return;
  }

  if (json.is_array()) {
    for (size_t i = 0; i < json.size(); ++i) {
      const auto token = "i" + std::to_string(i);
      const auto child_path = path.empty() ? token : path + "/" + token;
      publish_json_scalars(impl, json[i], prefix, child_path);
    }
  }
}

sensor_msgs::msg::Image make_image_msg(
  const cv::Mat & image,
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  cv::Mat publish_img = image;
  if (!publish_img.isContinuous()) {
    publish_img = publish_img.clone();
  }

  sensor_msgs::msg::Image msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.height = static_cast<uint32_t>(publish_img.rows);
  msg.width = static_cast<uint32_t>(publish_img.cols);
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(publish_img.step);

  if (publish_img.type() == CV_8UC3) {
    msg.encoding = "bgr8";
  } else if (publish_img.type() == CV_8UC1) {
    msg.encoding = "mono8";
  } else {
    // 目前调试视频通常是 bgr8/mono8。遇到其他格式时尽量不中断主流程。
    msg.encoding = "passthrough";
  }

  msg.data.assign(publish_img.datastart, publish_img.dataend);
  return msg;
}

}  // namespace

#else

class DebugRecorder::Impl
{
};

#endif  // TOOLS_HAS_ROS2_DEBUG_RECORDER

DebugRecorder::DebugRecorder() : DebugRecorder(Options{}) {}

DebugRecorder::DebugRecorder(const Options & options)
: options_(options), running_(true)
{
  options_.queue_size = std::max<size_t>(1, options_.queue_size);
  options_.image_stride = std::max<size_t>(1, options_.image_stride);

#ifdef TOOLS_HAS_ROS2_DEBUG_RECORDER
  impl_ = std::make_unique<Impl>(options_);
  enabled_ = impl_->node != nullptr;
  if (enabled_) {
    worker_ = std::thread(&DebugRecorder::worker_loop, this);
  }
#else
  (void)options_;
  logger()->warn("[DebugRecorder] ROS2 dependencies not found at build time, recorder disabled.");
  enabled_ = false;
#endif
}

DebugRecorder::~DebugRecorder()
{
  running_ = false;
  queue_cv_.notify_all();

  if (worker_.joinable()) {
    worker_.join();
  }
}

void DebugRecorder::record(const cv::Mat & image)
{
  record(data, image);
}

void DebugRecorder::record(const nlohmann::json & debug_data, const cv::Mat & image)
{
  if (!enabled_) {
    return;
  }

  const auto frame_index = submitted_frames_.fetch_add(1);
  const auto should_copy_image =
    options_.publish_image && !image.empty() && (frame_index % options_.image_stride == 0);

  Frame frame;
  frame.data = debug_data;

  // 把 frame 序号写进 JSON，方便离线检查 bag 中曲线和图像是否对齐。
  // 若调用者已经设置了 frame 字段，这里不覆盖。
  if (!frame.data.contains("frame")) {
    frame.data["frame"] = frame_index;
  }

  if (should_copy_image) {
    // cv::Mat 默认是浅拷贝。后台线程发布时主线程可能已经复用了图像内存，
    // 所以这里必须 clone。通过 image_stride 抽帧来控制这一步的成本。
    frame.image = image.clone();
    frame.has_image = true;
  }

  push_frame(std::move(frame));
}

void DebugRecorder::clear()
{
  data.clear();
}

void DebugRecorder::push_frame(Frame && frame)
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // 队列满时丢掉最旧的数据，保留最新调试状态。
    // 这是整个类“不影响主线程帧率”的核心策略。
    while (queue_.size() >= options_.queue_size) {
      queue_.pop_front();
    }

    queue_.push_back(std::move(frame));
  }
  queue_cv_.notify_one();
}

void DebugRecorder::worker_loop()
{
#ifdef TOOLS_HAS_ROS2_DEBUG_RECORDER
  while (running_) {
    Frame frame;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait_for(
        lock, std::chrono::milliseconds(5),
        [this] { return !running_ || !queue_.empty(); });

      if (!running_ && queue_.empty()) {
        break;
      }
      if (queue_.empty()) {
        rclcpp::spin_some(impl_->node);
        continue;
      }

      frame = std::move(queue_.front());
      queue_.pop_front();
    }

    const auto stamp = impl_->node->now();

    if (options_.publish_json && impl_->json_pub) {
      std_msgs::msg::String msg;
      msg.data = frame.data.dump();
      impl_->json_pub->publish(msg);
    }

    if (options_.publish_scalars) {
      publish_json_scalars(*impl_, frame.data, options_.scalar_topic_prefix, "");
    }

    if (frame.has_image && impl_->image_pub) {
      impl_->image_pub->publish(make_image_msg(frame.image, stamp, options_.frame_id));
    }

    // 发布端没有订阅回调也可以工作；这里 spin_some 主要用于保持节点事件处理顺畅。
    rclcpp::spin_some(impl_->node);
  }
#endif
}

}  // namespace tools
