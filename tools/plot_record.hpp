#ifndef TOOLS__PLOT_RECORD_HPP
#define TOOLS__PLOT_RECORD_HPP

#include <chrono>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

namespace tools
{
class PlotRecord
{
public:
  struct Options
  {
    std::string folder_path = "records";
    std::string file_prefix = "plot";
    std::string timestamp_key = "t";
    bool auto_timestamp = true;
    bool flush_each_record = false;
  };

  PlotRecord();
  explicit PlotRecord(const Options & options);
  PlotRecord(std::string folder_path, std::string file_prefix = "plot");
  ~PlotRecord();

  PlotRecord(const PlotRecord &) = delete;
  PlotRecord & operator=(const PlotRecord &) = delete;

  void record(const nlohmann::json & json);
  void record(
    const nlohmann::json & json, const std::chrono::steady_clock::time_point & timestamp);

  // Keep the same call shape as Plotter for quick replacement in debug code.
  void plot(const nlohmann::json & json);

  void flush();
  const std::string & path() const;

private:
  Options options_;
  std::string path_;
  std::ofstream writer_;
  std::chrono::steady_clock::time_point start_time_;
  std::mutex mutex_;
};
}  // namespace tools

#endif  // TOOLS__PLOT_RECORD_HPP
