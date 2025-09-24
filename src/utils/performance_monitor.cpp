/**
 * Performance Monitor Implementation
 * Real-time performance tracking
 */

#include "performance_monitor.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <sstream>
#include <iomanip>

#ifdef __linux__
#include <sys/resource.h>
#include <unistd.h>
#include <fstream>
#elif __APPLE__
#include <mach/mach.h>
#include <mach/task.h>
#include <sys/resource.h>
#elif _WIN32
#include <windows.h>
#include <psapi.h>
#endif

namespace Navigation {

// ScopedTimer Implementation

ScopedTimer::ScopedTimer(const std::string& name, std::function<void(double)> callback)
    : name_(name), callback_(callback), start_(steady_clock::now()) {
}

ScopedTimer::~ScopedTimer() {
    auto duration = steady_clock::now() - start_;
    double ms = duration_cast<microseconds>(duration).count() / 1000.0;
    if (callback_) {
        callback_(ms);
    }
}

// PerformanceMonitor Implementation

PerformanceMonitor::PerformanceMonitor(double total_budget_ms)
    : total_budget_ms_(total_budget_ms) {
    start_time_ = steady_clock::now();
    last_log_time_ = start_time_;
    last_resource_check_ = start_time_;

    {


        std::stringstream msg;


        msg << "Performance Monitor initialized with " << total_budget_ms_ << " ms budget";


        LOG_INFO(msg.str());


    }
}

PerformanceMonitor::PerformanceMonitor() {
    start_time_ = steady_clock::now();
    last_log_time_ = start_time_;
    last_resource_check_ = start_time_;

    {


        std::stringstream msg;


        msg << "Performance Monitor initialized with " << total_budget_ms_ << " ms budget (100 Hz)";


        LOG_INFO(msg.str());


    }
}

PerformanceMonitor::~PerformanceMonitor() {
    logDetailedStats();
}

void PerformanceMonitor::recordMetric(const std::string& name, double value) {
    // Generic metric recording
    // For now, map to existing methods based on name
    if (name == "ukf_time") {
        recordUKF(value);
    } else if (name == "rbpf_time") {
        recordRBPF(value);
    } else if (name == "ml_time") {
        recordML(value);
    } else if (name == "sensor_time") {
        recordSensor(value);
    } else if (name == "total_filter_time") {
        recordIteration(value);
    }
    // Other metrics can be logged directly
    {

        std::stringstream msg;

        msg << "Metric " << name << ": " << value;

        LOG_DEBUG(msg.str());

    }
}

void PerformanceMonitor::recordUKF(double ms) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    updateBuffer(ukf_times_, ms);
    ukf_count_++;
    
    if (!isWithinBudget(ms, ukf_budget_ms_)) {
        {

            std::stringstream msg;

            msg << "UKF exceeded budget: " << ms << " ms > " << ukf_budget_ms_ << " ms";

            LOG_WARN(msg.str());

        }
    }
}

void PerformanceMonitor::recordRBPF(double ms) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    updateBuffer(rbpf_times_, ms);
    rbpf_count_++;
    
    if (!isWithinBudget(ms, rbpf_budget_ms_)) {
        {

            std::stringstream msg;

            msg << "RBPF exceeded budget: " << ms << " ms > " << rbpf_budget_ms_ << " ms";

            LOG_WARN(msg.str());

        }
    }
}

void PerformanceMonitor::recordSensor(double ms) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    updateBuffer(sensor_times_, ms);
    sensor_count_++;
}

void PerformanceMonitor::recordML(double ms) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    updateBuffer(ml_times_, ms);
    ml_count_++;
    
    if (!isWithinBudget(ms, ml_budget_ms_)) {
        {

            std::stringstream msg;

            msg << "ML exceeded budget: " << ms << " ms > " << ml_budget_ms_ << " ms";

            LOG_WARN(msg.str());

        }
    }
}

void PerformanceMonitor::recordIteration(double total_ms) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    updateBuffer(total_times_, total_ms);
    iteration_count_++;
    
    if (!isWithinBudget(total_ms, total_budget_ms_)) {
        budget_violations_++;
        {

            std::stringstream msg;

            msg << "Iteration exceeded 10ms budget: " << total_ms << " ms";

            LOG_DEBUG(msg.str());

        }
    }
    
    // Periodic logging (every second)
    auto now = steady_clock::now();
    auto since_log = duration_cast<milliseconds>(now - last_log_time_).count();
    if (since_log > 1000) {
        logPerformance();
        last_log_time_ = now;
    }
    
    // Update resources every 5 seconds
    auto since_resource = duration_cast<seconds>(now - last_resource_check_).count();
    if (since_resource > 5) {
        updateResourceUsage();
        last_resource_check_ = now;
    }
}

ScopedTimer PerformanceMonitor::timeUKF() {
    return ScopedTimer("UKF", [this](double ms) { recordUKF(ms); });
}

ScopedTimer PerformanceMonitor::timeRBPF() {
    return ScopedTimer("RBPF", [this](double ms) { recordRBPF(ms); });
}

ScopedTimer PerformanceMonitor::timeSensor() {
    return ScopedTimer("Sensor", [this](double ms) { recordSensor(ms); });
}

ScopedTimer PerformanceMonitor::timeML() {
    return ScopedTimer("ML", [this](double ms) { recordML(ms); });
}

void PerformanceMonitor::updateBuffer(std::deque<double>& buffer, double value) {
    buffer.push_back(value);
    while (buffer.size() > BUFFER_SIZE) {
        buffer.pop_front();
    }
}

TimingStats PerformanceMonitor::computeStats(const std::deque<double>& buffer,
                                            uint64_t count, double budget) const {
    TimingStats stats;
    stats.count = count;
    
    if (buffer.empty()) {
        return stats;
    }
    
    // Copy for sorting (percentiles)
    std::vector<double> values(buffer.begin(), buffer.end());
    std::sort(values.begin(), values.end());
    
    // Basic statistics
    stats.min_ms = values.front();
    stats.max_ms = values.back();
    stats.total_ms = std::accumulate(values.begin(), values.end(), 0.0);
    stats.avg_ms = stats.total_ms / values.size();
    
    // Standard deviation
    double sq_sum = 0;
    for (double v : values) {
        sq_sum += (v - stats.avg_ms) * (v - stats.avg_ms);
    }
    stats.std_ms = sqrt(sq_sum / values.size());
    
    // Percentiles
    stats.p50_ms = computePercentile(values, 0.50);
    stats.p95_ms = computePercentile(values, 0.95);
    stats.p99_ms = computePercentile(values, 0.99);
    
    // Recent average (last 100 samples)
    size_t recent_count = std::min(size_t(100), buffer.size());
    double recent_sum = 0;
    auto it = buffer.rbegin();
    for (size_t i = 0; i < recent_count; ++i, ++it) {
        recent_sum += *it;
    }
    stats.recent_avg_ms = recent_sum / recent_count;
    
    // Check if meeting budget
    stats.meeting_budget = stats.p95_ms <= budget;
    
    return stats;
}

double PerformanceMonitor::computePercentile(std::vector<double> sorted_values, 
                                            double percentile) const {
    if (sorted_values.empty()) return 0;
    
    size_t idx = static_cast<size_t>(percentile * (sorted_values.size() - 1));
    return sorted_values[idx];
}

TimingStats PerformanceMonitor::getUKFStats() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return computeStats(ukf_times_, ukf_count_, ukf_budget_ms_);
}

TimingStats PerformanceMonitor::getRBPFStats() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return computeStats(rbpf_times_, rbpf_count_, rbpf_budget_ms_);
}

TimingStats PerformanceMonitor::getSensorStats() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return computeStats(sensor_times_, sensor_count_, sensor_budget_ms_);
}

TimingStats PerformanceMonitor::getMLStats() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return computeStats(ml_times_, ml_count_, ml_budget_ms_);
}

TimingStats PerformanceMonitor::getTotalStats() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return computeStats(total_times_, iteration_count_, total_budget_ms_);
}

SystemPerformance PerformanceMonitor::getSystemPerformance() const {
    SystemPerformance perf;
    
    // Component timings
    perf.ukf_timing = getUKFStats();
    perf.rbpf_timing = getRBPFStats();
    perf.sensor_timing = getSensorStats();
    perf.ml_timing = getMLStats();
    perf.total_timing = getTotalStats();
    
    // Resource usage
    perf.resources = getCurrentResources();
    
    // System health
    perf.total_iterations = iteration_count_;
    perf.budget_violations = budget_violations_;
    perf.timing_budget_usage = (perf.total_timing.avg_ms / total_budget_ms_) * 100.0;
    perf.real_time_capable = perf.total_timing.p95_ms <= total_budget_ms_;
    
    // Throughput
    auto elapsed = duration_cast<seconds>(steady_clock::now() - start_time_).count();
    if (elapsed > 0) {
        perf.iterations_per_second = static_cast<double>(iteration_count_) / elapsed;
    }
    
    return perf;
}

bool PerformanceMonitor::isWithinBudget(double ms, double budget_ms) const {
    return ms <= budget_ms;
}

bool PerformanceMonitor::checkTimingBudget(const std::string& component, double ms) {
    double budget = getBudget(component);
    bool within = isWithinBudget(ms, budget);
    
    if (!within) {
        {

            std::stringstream msg;

            msg << component << " timing violation: " << ms << " ms > " << budget << " ms budget";

            LOG_WARN(msg.str());

        }
        budget_violations_++;
    }
    
    return within;
}

bool PerformanceMonitor::isRealTimeCapable() const {
    auto total_stats = getTotalStats();
    return total_stats.p95_ms <= total_budget_ms_;
}

double PerformanceMonitor::getTimingMargin() const {
    auto total_stats = getTotalStats();
    return ((total_budget_ms_ - total_stats.avg_ms) / total_budget_ms_) * 100.0;
}

void PerformanceMonitor::setBudget(const std::string& component, double budget_ms) {
    if (component == "UKF") ukf_budget_ms_ = budget_ms;
    else if (component == "RBPF") rbpf_budget_ms_ = budget_ms;
    else if (component == "Sensor") sensor_budget_ms_ = budget_ms;
    else if (component == "ML") ml_budget_ms_ = budget_ms;
    else if (component == "Total") total_budget_ms_ = budget_ms;
}

double PerformanceMonitor::getBudget(const std::string& component) const {
    if (component == "UKF") return ukf_budget_ms_;
    if (component == "RBPF") return rbpf_budget_ms_;
    if (component == "Sensor") return sensor_budget_ms_;
    if (component == "ML") return ml_budget_ms_;
    if (component == "Total") return total_budget_ms_;
    return 0;
}

void PerformanceMonitor::logPerformance() const {
    auto perf = getSystemPerformance();
    
    LOG_INFO("=== Performance Summary ===");
    LOG_INFO("Iterations: " << perf.total_iterations 
             << " at " << std::fixed << std::setprecision(1) 
             << perf.iterations_per_second << " Hz");
    {

        std::stringstream msg;

        msg << "Total timing: " << std::fixed << std::setprecision(2;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Budget usage: " << std::fixed << std::setprecision(1;

        LOG_INFO(msg.str());

    }
    LOG_INFO("Violations: " << perf.budget_violations 
             << " (" << 100.0 * perf.budget_violations / perf.total_iterations << "%)");
    
    if (!perf.real_time_capable) {
        LOG_WARN("System NOT real-time capable! P95=" 
                << perf.total_timing.p95_ms << " ms > budget");
    }
}

void PerformanceMonitor::logDetailedStats() const {
    LOG_INFO("=== Detailed Performance Statistics ===");
    
    auto log_component = [](const std::string& name, const TimingStats& stats) {
        {

            std::stringstream msg;

            msg << name << " (" << stats.count << " calls):";

            LOG_INFO(msg.str());

        }
        {

            std::stringstream msg;

            msg << "  Min/Avg/Max: " << std::fixed << std::setprecision(2;

            LOG_INFO(msg.str());

        }
        LOG_INFO("  P50/P95/P99: " << stats.p50_ms << "/" 
                << stats.p95_ms << "/" << stats.p99_ms << " ms");
        {

            std::stringstream msg;

            msg << "  Std dev: " << stats.std_ms << " ms";

            LOG_INFO(msg.str());

        }
        {

            std::stringstream msg;

            msg << "  Meeting budget: " << (stats.meeting_budget ? "YES" : "NO");

            LOG_INFO(msg.str());

        }
    };
    
    log_component("UKF", getUKFStats());
    log_component("RBPF", getRBPFStats());
    log_component("Sensor", getSensorStats());
    log_component("ML", getMLStats());
    log_component("Total", getTotalStats());
    
    logResourceUsage();
}

void PerformanceMonitor::logResourceUsage() const {
    auto resources = getCurrentResources();
    
    LOG_INFO("=== Resource Usage ===");
    {

        std::stringstream msg;

        msg << "CPU: " << std::fixed << std::setprecision(1;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Memory: " << std::fixed << std::setprecision(1;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Threads: " << resources.thread_count;

        LOG_INFO(msg.str());

    }
}

void PerformanceMonitor::reset() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    ukf_times_.clear();
    rbpf_times_.clear();
    sensor_times_.clear();
    ml_times_.clear();
    total_times_.clear();
    
    iteration_count_ = 0;
    ukf_count_ = 0;
    rbpf_count_ = 0;
    sensor_count_ = 0;
    ml_count_ = 0;
    budget_violations_ = 0;
    
    start_time_ = steady_clock::now();
    last_log_time_ = start_time_;
}

ResourceUsage PerformanceMonitor::getCurrentResources() const {
#ifdef __linux__
    return getResourcesLinux();
#elif __APPLE__
    return getResourcesMacOS();
#elif _WIN32
    return getResourcesWindows();
#else
    return ResourceUsage();  // Default empty
#endif
}

void PerformanceMonitor::updateResourceUsage() {
    last_resources_ = getCurrentResources();
}

ResourceUsage PerformanceMonitor::getResourcesLinux() const {
    ResourceUsage usage;
    
#ifdef __linux__
    // CPU usage from /proc/self/stat
    std::ifstream stat_file("/proc/self/stat");
    if (stat_file.is_open()) {
        std::string line;
        std::getline(stat_file, line);
        // Parse CPU ticks (simplified)
        // In production, would track deltas for percentage
    }
    
    // Memory usage
    struct rusage r_usage;
    if (getrusage(RUSAGE_SELF, &r_usage) == 0) {
        usage.memory_mb = r_usage.ru_maxrss / 1024.0;  // Convert KB to MB
    }
    
    // Thread count from /proc/self/status
    std::ifstream status_file("/proc/self/status");
    if (status_file.is_open()) {
        std::string line;
        while (std::getline(status_file, line)) {
            if (line.find("Threads:") == 0) {
                sscanf(line.c_str(), "Threads: %lu", &usage.thread_count);
                break;
            }
        }
    }
#endif
    
    return usage;
}

ResourceUsage PerformanceMonitor::getResourcesMacOS() const {
    ResourceUsage usage;
    
#ifdef __APPLE__
    // Get task info
    task_t task = mach_task_self();
    struct task_basic_info info;
    mach_msg_type_number_t size = TASK_BASIC_INFO_COUNT;
    
    if (task_info(task, TASK_BASIC_INFO, (task_info_t)&info, &size) == KERN_SUCCESS) {
        usage.memory_mb = info.resident_size / (1024.0 * 1024.0);
        usage.thread_count = info.thread_count;
    }
    
    // CPU usage would require tracking time deltas
    struct rusage r_usage;
    if (getrusage(RUSAGE_SELF, &r_usage) == 0) {
        // r_usage.ru_utime and ru_stime contain user and system CPU time
        // Would need to track deltas to compute percentage
    }
#endif
    
    return usage;
}

ResourceUsage PerformanceMonitor::getResourcesWindows() const {
    ResourceUsage usage;
    
#ifdef _WIN32
    HANDLE process = GetCurrentProcess();
    PROCESS_MEMORY_COUNTERS pmc;
    
    if (GetProcessMemoryInfo(process, &pmc, sizeof(pmc))) {
        usage.memory_mb = pmc.WorkingSetSize / (1024.0 * 1024.0);
    }
    
    // CPU usage would require QueryProcessCycleTime and time tracking
#endif
    
    return usage;
}

} // namespace Navigation