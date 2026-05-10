#ifndef TOOLS__DEBUG_RECORDER_HPP
#define TOOLS__DEBUG_RECORDER_HPP

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

namespace tools
{

/**
 * @brief 异步 ROS2 调试发布器，用于配合 rosbag2 / Foxglove 录制和回放调试数据。
 *
 * 设计目标：
 * - 主线程只负责提交当前帧的图像和 JSON 数据，不负责 DDS 发布或磁盘写入。
 * - 队列满时丢弃旧调试帧，保证调试系统不会反向拖慢视觉主循环。
 * - 完整 JSON 会发布到 json_topic，便于离线还原每帧原始调试数据。
 * - JSON 中的数值字段会自动展开为 Float64 topic，便于 Foxglove Plot 直接画曲线。
 *
 * 注意：
 * - 这个类本身不写 rosbag 文件。实际录包仍然用外部命令：
 *   ros2 bag record /debug/image /debug/json /debug/...
 * - 调用本类前需要已经执行 rclcpp::init(argc, argv)。
 */
class DebugRecorder
{
public:
  struct Options
  {
    // ROS2 节点名。一个进程里如果创建多个 DebugRecorder，节点名需要不同。
    std::string node_name = "debug_recorder";

    // 图像 topic。建议发布 resize 后的调试图，而不是原始大图。
    std::string image_topic = "/debug/image";

    // 完整 JSON topic。消息类型为 std_msgs/msg/String。
    std::string json_topic = "/debug/json";

    // 数值 topic 前缀。例如 data["ekf"]["nis"] 会发布到 /debug/ekf/nis。
    std::string scalar_topic_prefix = "/debug";

    // 图像 frame_id，Foxglove 中用于标识图像坐标系。
    std::string frame_id = "debug_camera";

    // 队列最大长度。建议保持 1 或 2：调试数据宁可丢旧帧，也不要堆积。
    size_t queue_size = 1;

    // 图像抽帧倍率。1 表示每次 record 都提交图像；2 表示每 2 帧提交 1 帧图像。
    // 标量 JSON 不受此参数影响，仍然每次 record 都会提交。
    size_t image_stride = 1;

    // 是否发布图像 / 完整 JSON / 展开的数值 topic。
    bool publish_image = true;
    bool publish_json = true;
    bool publish_scalars = true;
  };

  DebugRecorder();
  explicit DebugRecorder(const Options & options);
  ~DebugRecorder();

  DebugRecorder(const DebugRecorder &) = delete;
  DebugRecorder & operator=(const DebugRecorder &) = delete;

  /**
   * @brief 兼容 Plotter 使用习惯的公开数据区。
   *
   * 推荐用法：
   *   debug_recorder.data["cmd_yaw"] = command.yaw;
   *   debug_recorder.data["target"]["x"] = x;
   *   debug_recorder.record(vis);
   *
   * 约定：data 只在主线程中修改。record() 会立即复制一份快照放入后台队列。
   */
  nlohmann::json data;

  /**
   * @brief 提交当前 data 和图像。
   *
   * 这个函数不会等待后台线程发布完成。若队列已满，会丢弃旧帧。
   * image 可以为空，此时只发布 JSON / scalar。
   */
  void record(const cv::Mat & image = cv::Mat());

  /**
   * @brief 直接提交外部构造好的 JSON，适合不想使用公开 data 成员的调用点。
   */
  void record(const nlohmann::json & debug_data, const cv::Mat & image = cv::Mat());

  /**
   * @brief 清空公开 data，通常在一帧结束后调用，避免上一帧字段残留。
   */
  void clear();

  /**
   * @brief 当前 ROS2 后端是否成功启用。
   */
  bool enabled() const { return enabled_; }

  class Impl;

private:
  struct Frame
  {
    nlohmann::json data;
    cv::Mat image;
    bool has_image = false;
  };

  void push_frame(Frame && frame);
  void worker_loop();

  Options options_;
  std::atomic<bool> enabled_{false};
  std::atomic<bool> running_{false};
  std::atomic<size_t> submitted_frames_{0};

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<Frame> queue_;
  std::thread worker_;

  std::unique_ptr<Impl> impl_;
};

}  // namespace tools

#endif  // TOOLS__DEBUG_RECORDER_HPP
