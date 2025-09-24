/**
 * Performance Monitor
 * Tracks system performance metrics and timing
 * Thread-safe with atomic operations
 */

#pragma once

#include <atomic>
#include <chrono>
#include <vector>
#include <deque>
#include <mutex>
#include <memory>
#include "logger.h"

namespace Navigation {

using namespace std::chrono;

/**
 * Timing statistics for a component
 */
struct TimingStats {
    double min_ms = 1e9;
    double max_ms = 0;
    double avg_ms = 0;
    double std_ms = 0;
    uint64_t count = 0;
    double total_ms = 0;
    
    // Percentiles
    double p50_ms = 0;  // Median
    double p95_ms = 0;
    double p99_ms = 0;
    
    // Recent performance
    double recent_avg_ms = 0;  // Last 100 samples
    bool meeting_budget = true;
};

/**
 * System resource usage
 */
struct ResourceUsage {
    double cpu_percent = 0.0;
    double memory_mb = 0.0;
    double memory_percent = 0.0;
    uint64_t thread_count = 0;
    double io_read_mb_s = 0.0;
    double io_write_mb_s = 0.0;
};

/**
 * Performance metrics for the entire system
 */
struct SystemPerformance {
    // Component timings
    TimingStats ukf_timing;
    TimingStats rbpf_timing;
    TimingStats sensor_timing;
    TimingStats ml_timing;
    TimingStats total_timing;
    
    // Resource usage
    ResourceUsage resources;
    
    // System health
    bool real_time_capable = true;
    double timing_budget_usage = 0.0;  // Percentage of 10ms budget
    uint64_t budget_violations = 0;
    uint64_t total_iterations = 0;
    
    // Throughput
    double iterations_per_second = 0.0;
    double data_throughput_mb_s = 0.0;
};

/**
 * Scoped timer for automatic timing
 */
class ScopedTimer {
private:
    std::string name_;
    steady_clock::time_point start_;
    std::function<void(double)> callback_;
    
public:
    ScopedTimer(const std::string& name, std::function<void(double)> callback);
    ~ScopedTimer();
};

/**
 * Performance Monitor
 */
class PerformanceMonitor {
private:
    // Timing buffers (circular)
    static constexpr size_t BUFFER_SIZE = 1000;
    std::deque<double> ukf_times_;
    std::deque<double> rbpf_times_;
    std::deque<double> sensor_times_;
    std::deque<double> ml_times_;
    std::deque<double> total_times_;
    
    // Thread-safe access
    mutable std::mutex buffer_mutex_;
    
    // Atomic counters for lock-free updates
    std::atomic<uint64_t> iteration_count_{0};
    std::atomic<uint64_t> ukf_count_{0};
    std::atomic<uint64_t> rbpf_count_{0};
    std::atomic<uint64_t> sensor_count_{0};
    std::atomic<uint64_t> ml_count_{0};
    std::atomic<uint64_t> budget_violations_{0};
    
    // Timing budget (milliseconds)
    double ukf_budget_ms_ = 5.0;
    double rbpf_budget_ms_ = 30.0;
    double sensor_budget_ms_ = 1.0;
    double ml_budget_ms_ = 10.0;
    double total_budget_ms_ = 10.0;  // 100 Hz target
    
    // Start time for rate calculation
    steady_clock::time_point start_time_;
    steady_clock::time_point last_log_time_;
    
    // Resource monitoring
    ResourceUsage last_resources_;
    steady_clock::time_point last_resource_check_;
    
public:
    PerformanceMonitor();
    PerformanceMonitor(double total_budget_ms);  // Constructor with budget
    ~PerformanceMonitor();
    
    // Record timings (thread-safe)
    void recordUKF(double ms);
    void recordRBPF(double ms);
    void recordSensor(double ms);
    void recordML(double ms);
    void recordIteration(double total_ms);
    
    // Scoped timing helpers
    ScopedTimer timeUKF();
    ScopedTimer timeRBPF();
    ScopedTimer timeSensor();
    ScopedTimer timeML();
    
    // Get statistics
    TimingStats getUKFStats() const;
    TimingStats getRBPFStats() const;
    TimingStats getSensorStats() const;
    TimingStats getMLStats() const;
    TimingStats getTotalStats() const;
    SystemPerformance getSystemPerformance() const;
    
    // Resource monitoring
    ResourceUsage getCurrentResources() const;
    void updateResourceUsage();
    
    // Budget checking
    bool isWithinBudget(double ms, double budget_ms) const;
    bool checkTimingBudget(const std::string& component, double ms);
    uint64_t getBudgetViolations() const { return budget_violations_.load(); }
    
    // Real-time capability check
    bool isRealTimeCapable() const;
    double getTimingMargin() const;  // Percentage margin to budget
    
    // Logging
    void logPerformance() const;
    void logDetailedStats() const;
    void logResourceUsage() const;
    
    // Reset statistics
    void reset();
    
    // Configuration
    void setBudget(const std::string& component, double budget_ms);
    double getBudget(const std::string& component) const;

    // Additional methods for navigation_system compatibility
    double getAverageTime() const { return getTotalStats().avg_ms; }
    double getMaxTime() const { return getTotalStats().max_ms; }
    void recordMetric(const std::string& name, double value);
    
private:
    // Update timing buffer
    void updateBuffer(std::deque<double>& buffer, double value);
    
    // Compute statistics from buffer
    TimingStats computeStats(const std::deque<double>& buffer, 
                             uint64_t count, double budget) const;
    
    // Compute percentiles
    double computePercentile(std::vector<double> sorted_values, double percentile) const;
    
    // Platform-specific resource monitoring
    ResourceUsage getResourcesLinux() const;
    ResourceUsage getResourcesMacOS() const;
    ResourceUsage getResourcesWindows() const;
};

/**
 * Global performance monitor singleton
 */
class GlobalPerformanceMonitor {
public:
    static PerformanceMonitor& getInstance() {
        static PerformanceMonitor instance;
        return instance;
    }
    
private:
    GlobalPerformanceMonitor() = default;
};

// Convenience macros for timing
#define PERF_TIME_UKF() ScopedTimer _timer_ukf("UKF", \
    [](double ms) { GlobalPerformanceMonitor::getInstance().recordUKF(ms); })

#define PERF_TIME_RBPF() ScopedTimer _timer_rbpf("RBPF", \
    [](double ms) { GlobalPerformanceMonitor::getInstance().recordRBPF(ms); })

#define PERF_TIME_SENSOR() ScopedTimer _timer_sensor("Sensor", \
    [](double ms) { GlobalPerformanceMonitor::getInstance().recordSensor(ms); })

#define PERF_TIME_ML() ScopedTimer _timer_ml("ML", \
    [](double ms) { GlobalPerformanceMonitor::getInstance().recordML(ms); })

} // namespace Navigation