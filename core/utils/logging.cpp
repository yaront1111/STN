#include "core/utils/logging.h"

#include <cstdlib>
#include <filesystem>
#include <iterator>
#include <vector>
#include <chrono>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

namespace aion {

std::shared_ptr<spdlog::logger> Logger::logger_ = nullptr;
std::shared_ptr<spdlog::sinks::sink> Logger::console_sink_ = nullptr;
std::shared_ptr<spdlog::sinks::sink> Logger::file_sink_ = nullptr;
bool Logger::initialized_ = false;
std::mutex Logger::mutex_;

static spdlog::level::level_enum parseLevel(const char* env) {
  if (!env) return spdlog::level::info;
  std::string s(env);
  for (auto& c : s) c = static_cast<char>(::tolower(c));
  if (s == "trace") return spdlog::level::trace;
  if (s == "debug") return spdlog::level::debug;
  if (s == "info") return spdlog::level::info;
  if (s == "warn" || s == "warning") return spdlog::level::warn;
  if (s == "error") return spdlog::level::err;
  if (s == "critical" || s == "fatal") return spdlog::level::critical;
  if (s == "off") return spdlog::level::off;
  return spdlog::level::info;
}

void Logger::init(const std::string& log_file,
                  spdlog::level::level_enum console_level,
                  size_t max_bytes,
                  size_t max_files,
                  int flush_every_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialized_) return;

  try {
    // Ensure log directory exists
    std::filesystem::path p(log_file);
    if (p.has_parent_path()) {
      std::filesystem::create_directories(p.parent_path());
    }

    // Sinks
    auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console->set_level(console_level);
    console->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

    auto rotating = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_file, max_bytes, max_files);
    rotating->set_level(spdlog::level::trace); // log everything to file
    rotating->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v");

    std::vector<spdlog::sink_ptr> sinks;
    sinks.reserve(2);
    sinks.emplace_back(console);
    sinks.emplace_back(rotating);

    logger_ = std::make_shared<spdlog::logger>("aion", sinks.begin(), sinks.end());
    logger_->set_level(console_level);             // effective minimum for logger
    logger_->flush_on(spdlog::level::warn);        // auto-flush on warn+

    spdlog::set_default_logger(logger_);
    spdlog::flush_every(std::chrono::seconds(std::max(1, flush_every_sec)));

    console_sink_ = console;
    file_sink_    = rotating;
    initialized_  = true;

    logger_->info("===========================================");
    logger_->info("AION Navigation — Logger initialized");
    logger_->info("Log file: {}", log_file);
    logger_->info("Console level: {}", spdlog::level::to_string_view(console_level));
    logger_->info("===========================================");
  } catch (const spdlog::spdlog_ex& ex) {
    // If spdlog fails, try a bare console fallback
    initialized_ = false;
    logger_.reset();
    console_sink_.reset();
    file_sink_.reset();
    fprintf(stderr, "Logger initialization failed: %s\n", ex.what());
  }
}

void Logger::initFromEnv() {
  const char* lf  = std::getenv("AION_LOG_FILE");
  const char* lvl = std::getenv("AION_LOG_LEVEL");
  init(lf ? std::string(lf) : std::string("logs/aion.log"),
       parseLevel(lvl));
}

void Logger::ensureInitializedUnlocked() {
  if (!initialized_) {
    // default init if user forgot
    init();
  }
}

std::shared_ptr<spdlog::logger> Logger::get() {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureInitializedUnlocked();
  return logger_;
}

void Logger::setLevel(spdlog::level::level_enum level) {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureInitializedUnlocked();
  logger_->set_level(level);
  if (console_sink_) console_sink_->set_level(level);
  // keep file sink at trace for full capture
  logger_->info("Logger level set to {}", spdlog::level::to_string_view(level));
}

void Logger::setConsoleLevel(spdlog::level::level_enum level) {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureInitializedUnlocked();
  if (console_sink_) console_sink_->set_level(level);
  logger_->info("Console sink level set to {}", spdlog::level::to_string_view(level));
}

void Logger::setFileLevel(spdlog::level::level_enum level) {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureInitializedUnlocked();
  if (file_sink_) file_sink_->set_level(level);
  logger_->info("File sink level set to {}", spdlog::level::to_string_view(level));
}

void Logger::rotate() {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureInitializedUnlocked();
  // Note: rotate_() is private in newer spdlog versions
  // Rotation happens automatically based on file size
  logger_->flush();
  logger_->info("Log flush requested (rotation is automatic based on size)");
}

void Logger::enableBacktrace(size_t n_messages) {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureInitializedUnlocked();
  logger_->enable_backtrace(n_messages);
  logger_->info("Backtrace enabled ({} messages)", n_messages);
}

void Logger::dumpBacktrace() {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureInitializedUnlocked();
  logger_->dump_backtrace();
  logger_->info("Backtrace dumped");
}

void Logger::flush() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) return;
  logger_->flush();
}

void Logger::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) return;
  logger_->info("AION logger shutdown");
  logger_->flush();
  spdlog::shutdown();
  logger_.reset();
  console_sink_.reset();
  file_sink_.reset();
  initialized_ = false;
}

}  // namespace aion
