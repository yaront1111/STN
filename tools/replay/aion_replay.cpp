#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

#include "core/utils/binary_reader.h"
#include "core/utils/logging.h"
#include "core/utils/time_utils.h"
#include "aion_messages.pb.h"

void printUsage(const char* program_name) {
  std::cout << "AION Binary Log Replay Tool\n";
  std::cout << "Usage: " << program_name << " [options]\n";
  std::cout << "Options:\n";
  std::cout << "  --input <file>    Input binary log file (required)\n";
  std::cout << "  --speed <factor>  Playback speed (default: 1.0)\n";
  std::cout << "  --stats           Show statistics only\n";
  std::cout << "  --filter <type>   Filter by message type (IMU|STATE)\n";
  std::cout << "  --start <time>    Start time offset in seconds\n";
  std::cout << "  --help            Show this help message\n";
}

struct Args {
  std::string input_file;
  double speed = 1.0;
  bool stats_only = false;
  std::string filter_type;
  double start_time = 0.0;
};

Args parseArgs(int argc, char* argv[]) {
  Args args;

  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);

    if (arg == "--input" && i + 1 < argc) {
      args.input_file = argv[++i];
    } else if (arg == "--speed" && i + 1 < argc) {
      args.speed = std::stod(argv[++i]);
    } else if (arg == "--stats") {
      args.stats_only = true;
    } else if (arg == "--filter" && i + 1 < argc) {
      args.filter_type = argv[++i];
    } else if (arg == "--start" && i + 1 < argc) {
      args.start_time = std::stod(argv[++i]);
    } else if (arg == "--help") {
      printUsage(argv[0]);
      exit(0);
    }
  }

  if (args.input_file.empty()) {
    std::cerr << "Error: --input file required\n\n";
    printUsage(argv[0]);
    exit(1);
  }

  return args;
}

void printStats(const aion::BinaryReader& reader) {
  auto stats = const_cast<aion::BinaryReader&>(reader).computeStats();

  std::cout << "\n=== Log File Statistics ===\n";
  std::cout << "Total frames:  " << stats.total_frames << "\n";
  std::cout << "IMU messages:  " << stats.imu_count << "\n";
  std::cout << "State messages: " << stats.state_count << "\n";
  std::cout << "Duration:      " << std::fixed << std::setprecision(2)
            << stats.duration << " seconds\n";
  std::cout << "Time range:    [" << std::fixed << std::setprecision(6)
            << stats.min_timestamp << ", " << stats.max_timestamp << "]\n";

  if (stats.imu_count > 0) {
    double imu_rate = stats.imu_count / stats.duration;
    std::cout << "IMU rate:      " << std::fixed << std::setprecision(1)
              << imu_rate << " Hz\n";
  }

  if (stats.state_count > 0) {
    double state_rate = stats.state_count / stats.duration;
    std::cout << "State rate:    " << std::fixed << std::setprecision(1)
              << state_rate << " Hz\n";
  }
}

void replayLog(aion::BinaryReader& reader, const Args& args) {
  const auto* header = reader.getHeader();
  if (!header) {
    std::cerr << "No header found in log file\n";
    return;
  }

  std::cout << "\n=== Starting Replay ===\n";
  std::cout << "Speed: " << args.speed << "x\n";
  std::cout << "Origin: lat=" << header->origin_lat()
            << ", lon=" << header->origin_lon()
            << ", alt=" << header->origin_alt() << "\n\n";

  // Seek to start time if specified
  if (args.start_time > 0) {
    reader.seekToTime(header->start_timestamp() + args.start_time);
  }

  auto start_wall_time = std::chrono::steady_clock::now();
  double first_log_time = -1.0;
  uint64_t frame_count = 0;
  uint64_t imu_count = 0;
  uint64_t state_count = 0;

  while (reader.hasNext()) {
    auto frame = reader.readNext();
    if (!frame) break;

    // Apply filter if specified
    if (!args.filter_type.empty()) {
      if (args.filter_type == "IMU" && frame->type() != aion::msgs::LogFrame::IMU) {
        continue;
      }
      if (args.filter_type == "STATE" && frame->type() != aion::msgs::LogFrame::STATE) {
        continue;
      }
    }

    // Track first timestamp
    if (first_log_time < 0) {
      first_log_time = frame->timestamp();
    }

    // Calculate timing for realistic playback
    double log_elapsed = frame->timestamp() - first_log_time;
    double target_wall_elapsed = log_elapsed / args.speed;

    auto current_wall_time = std::chrono::steady_clock::now();
    auto actual_wall_elapsed = std::chrono::duration<double>(
        current_wall_time - start_wall_time).count();

    // Sleep if we're ahead of schedule
    if (actual_wall_elapsed < target_wall_elapsed) {
      double sleep_time = target_wall_elapsed - actual_wall_elapsed;
      std::this_thread::sleep_for(
          std::chrono::duration<double>(sleep_time));
    }

    // Process and display frame
    frame_count++;

    switch (frame->type()) {
      case aion::msgs::LogFrame::IMU: {
        aion::msgs::IMUMessage imu_msg;
        if (imu_msg.ParseFromString(frame->data())) {
          imu_count++;
          std::cout << "[IMU] t=" << std::fixed << std::setprecision(3)
                    << log_elapsed << "s"
                    << " gyro=[" << imu_msg.gyro_x() << ", "
                    << imu_msg.gyro_y() << ", " << imu_msg.gyro_z() << "]"
                    << " accel=[" << imu_msg.accel_x() << ", "
                    << imu_msg.accel_y() << ", " << imu_msg.accel_z() << "]\n";
        }
        break;
      }

      case aion::msgs::LogFrame::STATE: {
        aion::msgs::StateMessage state_msg;
        if (state_msg.ParseFromString(frame->data())) {
          state_count++;
          std::cout << "[STATE] t=" << std::fixed << std::setprecision(3)
                    << log_elapsed << "s"
                    << " pos=[" << state_msg.pos_n() << ", "
                    << state_msg.pos_e() << ", " << state_msg.pos_d() << "]"
                    << " vel=[" << state_msg.vel_n() << ", "
                    << state_msg.vel_e() << ", " << state_msg.vel_d() << "]\n";
        }
        break;
      }

      default:
        break;
    }

    // Show progress every second
    if (frame_count % 200 == 0) {
      std::cout << "--- Processed " << frame_count << " frames"
                << " (IMU: " << imu_count << ", State: " << state_count << ")"
                << " ---\n";
    }
  }

  std::cout << "\n=== Replay Complete ===\n";
  std::cout << "Total frames replayed: " << frame_count << "\n";
  std::cout << "IMU messages: " << imu_count << "\n";
  std::cout << "State messages: " << state_count << "\n";
}

int main(int argc, char* argv[]) {
  // Initialize logger (console only for replay tool)
  aion::Logger::init("/tmp/aion_replay.log", spdlog::level::info);

  // Parse arguments
  Args args = parseArgs(argc, argv);

  // Open log file
  aion::BinaryReader reader;
  if (!reader.open(args.input_file)) {
    LOG_ERROR("Failed to open log file: {}", args.input_file);
    return 1;
  }

  LOG_INFO("Opened log file: {}", args.input_file);

  // Show statistics
  printStats(reader);

  // Replay if not stats-only mode
  if (!args.stats_only) {
    replayLog(reader, args);
  }

  return 0;
}