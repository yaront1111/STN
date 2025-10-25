#pragma once
#include <fstream>
#include <sstream>
#include <string>

namespace chimera {
namespace utils {

/**
 * @brief Simple JSONL (JSON Lines) telemetry logger
 * Each line is a complete JSON object for easy parsing/streaming
 */
class Telemetry {
 public:
  /**
   * @param path Output file path (e.g., "logs/chimera.jsonl")
   */
  explicit Telemetry(const std::string& path) {
    ofs_.open(path, std::ios::out | std::ios::trunc);
    if (!ofs_.is_open()) {
      throw std::runtime_error("Failed to open telemetry file: " + path);
    }
  }

  ~Telemetry() {
    if (ofs_.is_open()) {
      ofs_.close();
    }
  }

  /**
   * @brief Write a line to the telemetry file
   * @param line String to write (should be valid JSON)
   */
  void writeLine(const std::string& line) {
    if (ofs_.is_open()) {
      ofs_ << line << "\n";
      ofs_.flush();  // Ensure immediate write for real-time monitoring
    }
  }

  /**
   * @brief Helper to write formatted JSON
   * @param json_content Pre-formatted JSON string
   */
  template <typename T>
  void log(const T& json_content) {
    std::ostringstream oss;
    oss << json_content;
    writeLine(oss.str());
  }

  bool isOpen() const { return ofs_.is_open(); }

 private:
  std::ofstream ofs_;
};

}  // namespace utils
}  // namespace chimera
