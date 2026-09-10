#pragma once

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <system_error>

namespace sim2real_common
{

class EventLogger
{
public:
  static std::string makeRunId()
  {
    const auto now = std::chrono::system_clock::now();
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()).count() % 1000;
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    const std::tm tm_value = toLocalTm(now_time);

    std::ostringstream oss;
    oss << "run_"
        << std::put_time(&tm_value, "%Y-%m-%d_%H-%M-%S")
        << "_"
        << std::setw(3) << std::setfill('0') << millis;
    return oss.str();
  }

  void configure(const std::string & log_dir, const std::string & file_stem)
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    log_dir_ = log_dir;
    file_stem_ = file_stem;
  }

  std::string componentLogPath() const
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    return componentLogPathUnlocked();
  }

  std::string timelineLogPath() const
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    return timelineLogPathUnlocked();
  }

  std::string summaryLogPath() const
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    return summaryLogPathUnlocked();
  }

  void log(
    const std::string & level,
    const std::string & component,
    const std::string & event,
    const std::string & message)
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    if (!isConfiguredUnlocked()) {
      return;
    }

    const std::string line = buildLogLine(level, component, event, message);
    appendLineUnlocked(componentLogPathUnlocked(), line);
    appendLineUnlocked(timelineLogPathUnlocked(), line);
  }

  void logSummary(const std::string & component, const std::string & message)
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    if (!isConfiguredUnlocked()) {
      return;
    }

    const std::string line = buildLogLine("SUMMARY", component, "session_summary", message);
    appendLineUnlocked(summaryLogPathUnlocked(), line);
    appendLineUnlocked(timelineLogPathUnlocked(), line);
  }

private:
  static std::tm toLocalTm(std::time_t now_time)
  {
    std::tm tm_value{};
#ifdef _WIN32
    localtime_s(&tm_value, &now_time);
#else
    localtime_r(&now_time, &tm_value);
#endif
    return tm_value;
  }

  static std::string formatTimestamp()
  {
    const auto now = std::chrono::system_clock::now();
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()).count() % 1000;
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    const std::tm tm_value = toLocalTm(now_time);

    std::ostringstream oss;
    oss << std::put_time(&tm_value, "%Y-%m-%d %H:%M:%S")
        << '.'
        << std::setw(3) << std::setfill('0') << millis;
    return oss.str();
  }

  static std::string buildLogLine(
    const std::string & level,
    const std::string & component,
    const std::string & event,
    const std::string & message)
  {
    std::ostringstream oss;
    oss << "[" << formatTimestamp() << "]"
        << "[" << level << "]"
        << "[" << component << "]"
        << "[" << event << "] "
        << message;
    return oss.str();
  }

  bool isConfiguredUnlocked() const
  {
    return !log_dir_.empty() && !file_stem_.empty();
  }

  void appendLineUnlocked(const std::string & path, const std::string & line) const
  {
    std::error_code ec;
    std::filesystem::create_directories(log_dir_, ec);

    std::ofstream stream(path, std::ios::app);
    if (!stream.is_open()) {
      return;
    }
    stream << line << '\n';
  }

  std::string componentLogPathUnlocked() const
  {
    return (std::filesystem::path(log_dir_) / (file_stem_ + ".log")).string();
  }

  std::string timelineLogPathUnlocked() const
  {
    return (std::filesystem::path(log_dir_) / "timeline.log").string();
  }

  std::string summaryLogPathUnlocked() const
  {
    return (std::filesystem::path(log_dir_) / "summary.log").string();
  }

  mutable std::mutex mutex_;
  std::string log_dir_;
  std::string file_stem_;
};

}  // namespace sim2real_common
