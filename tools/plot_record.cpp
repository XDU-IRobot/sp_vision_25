#include "plot_record.hpp"

#include <fmt/chrono.h>
#include <fmt/format.h>

#include <filesystem>
#include <utility>

#include "math_tools.hpp"

namespace tools
{
PlotRecord::PlotRecord() : PlotRecord(Options{}) {}

PlotRecord::PlotRecord(const Options & options) : options_(options)
{
  start_time_ = std::chrono::steady_clock::now();

  std::filesystem::create_directories(options_.folder_path);
  auto file_name = fmt::format(
    "{}_{:%Y-%m-%d_%H-%M-%S}.jsonl", options_.file_prefix, std::chrono::system_clock::now());
  path_ = (std::filesystem::path(options_.folder_path) / file_name).string();
  writer_.open(path_);
}

PlotRecord::PlotRecord(std::string folder_path, std::string file_prefix)
: PlotRecord([&] {
    Options options;
    options.folder_path = std::move(folder_path);
    options.file_prefix = std::move(file_prefix);
    return options;
  }())
{
}

PlotRecord::~PlotRecord()
{
  flush();
  if (writer_.is_open()) writer_.close();
}

void PlotRecord::record(const nlohmann::json & json)
{
  record(json, std::chrono::steady_clock::now());
}

void PlotRecord::record(
  const nlohmann::json & json, const std::chrono::steady_clock::time_point & timestamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!writer_.is_open()) return;

  nlohmann::json line = json.is_object() ? json : nlohmann::json{{"value", json}};

  if (
    options_.auto_timestamp && !options_.timestamp_key.empty() &&
    !line.contains(options_.timestamp_key)) {
    line[options_.timestamp_key] = tools::delta_time(timestamp, start_time_);
  }

  writer_ << line.dump() << '\n';
  if (options_.flush_each_record) writer_.flush();
}

void PlotRecord::plot(const nlohmann::json & json) { record(json); }

void PlotRecord::flush()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (writer_.is_open()) writer_.flush();
}

const std::string & PlotRecord::path() const { return path_; }
}  // namespace tools
