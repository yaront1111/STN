#pragma once

#include <memory>
#include <string>
#include <spdlog/spdlog.h>

namespace aion {

/**
 * Global logger wrapper (thread-safe).
 *
 * Env overrides (optional):
 *  - AION_LOG_FILE=/path/to/aion.log
 *  - AION_LOG_LEVEL=trace|debug|info|warn|error|critical|off
 */
class Logger {
 public:
  // Initialize logger (safe to call multiple times).
  // max_bytes: rotating size in bytes; max_files: number of rotated files to keep.
  static void init(const std::string& log_file = "logs/aion.log",
                   spdlog::level::level_enum console_level = spdlog::level::info,
                   size_t max_bytes = 10u * 1024u * 1024u,
                   size_t max_files = 3,
                   int flush_every_sec = 5);

  // Initialize using environment variables (see header docs). Falls back to init().
  static void initFromEnv();

  // Fetch the logger (auto-inits with defaults if needed).
  static std::shared_ptr<spdlog::logger> get();

  // Change levels at runtime
  static void setLevel(spdlog::level::level_enum level);        // sets logger level
  static void setConsoleLevel(spdlog::level::level_enum level); // console sink only
  static void setFileLevel(spdlog::level::level_enum level);    // file sink only

  // Force log rotation now (useful per run).
  static void rotate();

  // Backtrace ring buffer (captures recent messages for post-mortem)
  static void enableBacktrace(size_t n_messages = 1024);
  static void dumpBacktrace();

  // Flush and shutdown
  static void flush();
  static void shutdown();

 private:
  static void ensureInitializedUnlocked();  // internal

  static std::shared_ptr<spdlog::logger> logger_;
  static std::shared_ptr<spdlog::sinks::sink> console_sink_;
  static std::shared_ptr<spdlog::sinks::sink> file_sink_;
  static bool initialized_;
  static std::mutex mutex_;
};

// Convenience macros (compile-time filtered by SPDLOG_ACTIVE_LEVEL)
#define LOG_TRACE(...)    SPDLOG_LOGGER_TRACE(aion::Logger::get(), __VA_ARGS__)
#define LOG_DEBUG(...)    SPDLOG_LOGGER_DEBUG(aion::Logger::get(), __VA_ARGS__)
#define LOG_INFO(...)     SPDLOG_LOGGER_INFO(aion::Logger::get(), __VA_ARGS__)
#define LOG_WARN(...)     SPDLOG_LOGGER_WARN(aion::Logger::get(), __VA_ARGS__)
#define LOG_ERROR(...)    SPDLOG_LOGGER_ERROR(aion::Logger::get(), __VA_ARGS__)
#define LOG_CRITICAL(...) SPDLOG_LOGGER_CRITICAL(aion::Logger::get(), __VA_ARGS__)

}  // namespace aion
