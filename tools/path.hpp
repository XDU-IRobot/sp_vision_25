#ifndef TOOLS__PATH_HPP
#define TOOLS__PATH_HPP

#include <filesystem>
#include <string>

namespace tools
{
inline std::string resolve_path_from_config(
  const std::string & config_path,
  const std::string & raw_path)
{
  namespace fs = std::filesystem;

  if (raw_path.empty()) {
    return raw_path;
  }

  fs::path path(raw_path);
  if (path.is_absolute()) {
    return path.string();
  }

  if (fs::exists(path)) {
    return fs::weakly_canonical(path).string();
  }

  fs::path cfg(config_path);
  fs::path candidate = cfg.parent_path() / path;
  if (fs::exists(candidate)) {
    return fs::weakly_canonical(candidate).string();
  }

  // Return config-relative normalized path so downstream errors are explicit.
  return candidate.lexically_normal().string();
}
}  // namespace tools

#endif  // TOOLS__PATH_HPP
