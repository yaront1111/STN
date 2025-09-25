/**
 * Navigation System Grader
 * Automatically evaluates system performance and assigns grades
 */

#pragma once

#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace Navigation {

struct TestResult {
    std::string name;
    bool passed;
    double execution_time_ms;
    std::string failure_reason;
};

struct PerformanceMetrics {
    double mean_prediction_time_ms = 0;
    double p99_prediction_time_ms = 0;
    double mean_update_time_ms = 0;
    double max_memory_usage_mb = 0;
    double sustained_rate_hz = 0;
    bool meets_realtime_requirements = false;
};

struct CoverageMetrics {
    int total_tests = 0;
    int passed_tests = 0;
    int failed_tests = 0;
    double pass_rate = 0;

    // Category breakdown
    int ukf_tests_passed = 0;
    int ukf_tests_total = 0;
    int rbpf_tests_passed = 0;
    int rbpf_tests_total = 0;
    int integration_tests_passed = 0;
    int integration_tests_total = 0;
    int performance_tests_passed = 0;
    int performance_tests_total = 0;
};

struct CodeQualityMetrics {
    bool has_memory_leaks = false;
    bool has_race_conditions = false;
    int compiler_warnings = 0;
    double code_coverage_percent = 0;
    bool follows_coding_standards = true;
};

struct SystemGrade {
    // Overall grade (A+, A, A-, B+, B, B-, C+, C, C-, D, F)
    std::string letter_grade;
    double numeric_score;  // 0-100

    // Component scores (0-100)
    double functionality_score;
    double performance_score;
    double reliability_score;
    double test_coverage_score;
    double code_quality_score;

    // Detailed breakdown
    CoverageMetrics coverage;
    PerformanceMetrics performance;
    CodeQualityMetrics quality;

    // Recommendations
    std::vector<std::string> strengths;
    std::vector<std::string> improvements_needed;
    std::vector<std::string> critical_issues;

    // Timestamp
    std::chrono::system_clock::time_point evaluation_time;
};

class NavigationGrader {
public:
    NavigationGrader() = default;

    // Main grading function
    SystemGrade gradeSystem(const std::vector<TestResult>& test_results,
                           const PerformanceMetrics& perf_metrics);

    // Individual component graders
    double gradeFunctionality(const CoverageMetrics& coverage);
    double gradePerformance(const PerformanceMetrics& perf);
    double gradeReliability(const CoverageMetrics& coverage);
    double gradeTestCoverage(const CoverageMetrics& coverage);
    double gradeCodeQuality(const CodeQualityMetrics& quality);

    // Generate grade report
    std::string generateReport(const SystemGrade& grade);
    void saveReportToFile(const SystemGrade& grade, const std::string& filename);

    // Grade tracking over time
    void recordGrade(const SystemGrade& grade);
    std::vector<SystemGrade> getGradeHistory() const { return grade_history_; }
    std::string generateTrendReport() const;

    // Helper functions (public for test runner access)
    std::string calculateLetterGrade(double numeric_score);

private:
    std::vector<SystemGrade> grade_history_;
    CoverageMetrics analyzeCoverage(const std::vector<TestResult>& results);
    void identifyStrengths(SystemGrade& grade);
    void identifyImprovements(SystemGrade& grade);
    void identifyCriticalIssues(SystemGrade& grade);

    // Scoring weights
    const double FUNCTIONALITY_WEIGHT = 0.30;
    const double PERFORMANCE_WEIGHT = 0.25;
    const double RELIABILITY_WEIGHT = 0.20;
    const double COVERAGE_WEIGHT = 0.15;
    const double QUALITY_WEIGHT = 0.10;

    // Grading thresholds
    const double A_PLUS_THRESHOLD = 97.0;
    const double A_THRESHOLD = 93.0;
    const double A_MINUS_THRESHOLD = 90.0;
    const double B_PLUS_THRESHOLD = 87.0;
    const double B_THRESHOLD = 83.0;
    const double B_MINUS_THRESHOLD = 80.0;
    const double C_PLUS_THRESHOLD = 77.0;
    const double C_THRESHOLD = 73.0;
    const double C_MINUS_THRESHOLD = 70.0;
    const double D_THRESHOLD = 60.0;
};

// Inline implementations

inline SystemGrade NavigationGrader::gradeSystem(
    const std::vector<TestResult>& test_results,
    const PerformanceMetrics& perf_metrics) {

    SystemGrade grade;
    grade.evaluation_time = std::chrono::system_clock::now();

    // Analyze test coverage
    grade.coverage = analyzeCoverage(test_results);
    grade.performance = perf_metrics;

    // Calculate component scores
    grade.functionality_score = gradeFunctionality(grade.coverage);
    grade.performance_score = gradePerformance(perf_metrics);
    grade.reliability_score = gradeReliability(grade.coverage);
    grade.test_coverage_score = gradeTestCoverage(grade.coverage);
    grade.code_quality_score = gradeCodeQuality(grade.quality);

    // Calculate overall score
    grade.numeric_score =
        grade.functionality_score * FUNCTIONALITY_WEIGHT +
        grade.performance_score * PERFORMANCE_WEIGHT +
        grade.reliability_score * RELIABILITY_WEIGHT +
        grade.test_coverage_score * COVERAGE_WEIGHT +
        grade.code_quality_score * QUALITY_WEIGHT;

    grade.letter_grade = calculateLetterGrade(grade.numeric_score);

    // Generate feedback
    identifyStrengths(grade);
    identifyImprovements(grade);
    identifyCriticalIssues(grade);

    return grade;
}

inline double NavigationGrader::gradeFunctionality(const CoverageMetrics& coverage) {
    double score = 100.0;

    // Core UKF functionality (40% of functionality)
    double ukf_rate = coverage.ukf_tests_total > 0 ?
        static_cast<double>(coverage.ukf_tests_passed) / coverage.ukf_tests_total : 0;
    score -= (1.0 - ukf_rate) * 40.0;

    // RBPF functionality (20% of functionality)
    double rbpf_rate = coverage.rbpf_tests_total > 0 ?
        static_cast<double>(coverage.rbpf_tests_passed) / coverage.rbpf_tests_total : 0;
    score -= (1.0 - rbpf_rate) * 20.0;

    // Integration tests (40% of functionality)
    double integration_rate = coverage.integration_tests_total > 0 ?
        static_cast<double>(coverage.integration_tests_passed) / coverage.integration_tests_total : 0;
    score -= (1.0 - integration_rate) * 40.0;

    return std::max(0.0, score);
}

inline double NavigationGrader::gradePerformance(const PerformanceMetrics& perf) {
    double score = 100.0;

    // Prediction time (<1ms = 100%, >5ms = 0%)
    if (perf.mean_prediction_time_ms > 1.0) {
        score -= std::min(30.0, (perf.mean_prediction_time_ms - 1.0) * 7.5);
    }

    // P99 prediction time (<5ms = 100%, >20ms = 0%)
    if (perf.p99_prediction_time_ms > 5.0) {
        score -= std::min(20.0, (perf.p99_prediction_time_ms - 5.0) * 1.33);
    }

    // Update time (<2ms = 100%, >10ms = 0%)
    if (perf.mean_update_time_ms > 2.0) {
        score -= std::min(20.0, (perf.mean_update_time_ms - 2.0) * 2.5);
    }

    // Sustained rate (>100Hz = 100%, <50Hz = 0%)
    if (perf.sustained_rate_hz < 100.0) {
        score -= std::min(20.0, (100.0 - perf.sustained_rate_hz) * 0.4);
    }

    // Real-time requirements
    if (!perf.meets_realtime_requirements) {
        score -= 10.0;
    }

    return std::max(0.0, score);
}

inline double NavigationGrader::gradeReliability(const CoverageMetrics& coverage) {
    double score = 100.0;

    // Overall pass rate
    if (coverage.pass_rate < 1.0) {
        score -= (1.0 - coverage.pass_rate) * 50.0;
    }

    // Critical test failures (UKF tests are most critical)
    double ukf_pass_rate = coverage.ukf_tests_total > 0 ?
        static_cast<double>(coverage.ukf_tests_passed) / coverage.ukf_tests_total : 0;
    if (ukf_pass_rate < 1.0) {
        score -= (1.0 - ukf_pass_rate) * 30.0;
    }

    // Integration test failures
    double integration_pass_rate = coverage.integration_tests_total > 0 ?
        static_cast<double>(coverage.integration_tests_passed) / coverage.integration_tests_total : 0;
    if (integration_pass_rate < 1.0) {
        score -= (1.0 - integration_pass_rate) * 20.0;
    }

    return std::max(0.0, score);
}

inline double NavigationGrader::gradeTestCoverage(const CoverageMetrics& coverage) {
    double score = 100.0;

    // Total number of tests
    if (coverage.total_tests < 50) {
        score -= 20.0;
    } else if (coverage.total_tests < 100) {
        score -= 10.0;
    } else if (coverage.total_tests < 150) {
        score -= 5.0;
    }

    // Test distribution
    bool has_ukf_tests = coverage.ukf_tests_total > 10;
    bool has_rbpf_tests = coverage.rbpf_tests_total > 5;
    bool has_integration_tests = coverage.integration_tests_total > 5;
    bool has_performance_tests = coverage.performance_tests_total > 5;

    if (!has_ukf_tests) score -= 25.0;
    if (!has_rbpf_tests) score -= 15.0;
    if (!has_integration_tests) score -= 20.0;
    if (!has_performance_tests) score -= 15.0;

    return std::max(0.0, score);
}

inline double NavigationGrader::gradeCodeQuality(const CodeQualityMetrics& quality) {
    double score = 100.0;

    if (quality.has_memory_leaks) score -= 30.0;
    if (quality.has_race_conditions) score -= 30.0;
    if (quality.compiler_warnings > 0) {
        score -= std::min(20.0, quality.compiler_warnings * 2.0);
    }
    if (quality.code_coverage_percent < 70.0) {
        score -= (70.0 - quality.code_coverage_percent) * 0.3;
    }
    if (!quality.follows_coding_standards) score -= 10.0;

    return std::max(0.0, score);
}

inline std::string NavigationGrader::calculateLetterGrade(double numeric_score) {
    if (numeric_score >= A_PLUS_THRESHOLD) return "A+";
    if (numeric_score >= A_THRESHOLD) return "A";
    if (numeric_score >= A_MINUS_THRESHOLD) return "A-";
    if (numeric_score >= B_PLUS_THRESHOLD) return "B+";
    if (numeric_score >= B_THRESHOLD) return "B";
    if (numeric_score >= B_MINUS_THRESHOLD) return "B-";
    if (numeric_score >= C_PLUS_THRESHOLD) return "C+";
    if (numeric_score >= C_THRESHOLD) return "C";
    if (numeric_score >= C_MINUS_THRESHOLD) return "C-";
    if (numeric_score >= D_THRESHOLD) return "D";
    return "F";
}

inline std::string NavigationGrader::generateReport(const SystemGrade& grade) {
    std::stringstream report;

    report << "\n";
    report << "========================================\n";
    report << "   NAVIGATION SYSTEM GRADE REPORT\n";
    report << "========================================\n\n";

    // Overall Grade
    report << "OVERALL GRADE: " << grade.letter_grade
           << " (" << std::fixed << std::setprecision(1)
           << grade.numeric_score << "%)\n";
    report << "----------------------------------------\n\n";

    // Component Scores
    report << "COMPONENT SCORES:\n";
    report << "  Functionality:    " << std::setw(6) << std::setprecision(1)
           << grade.functionality_score << "% (30% weight)\n";
    report << "  Performance:      " << std::setw(6)
           << grade.performance_score << "% (25% weight)\n";
    report << "  Reliability:      " << std::setw(6)
           << grade.reliability_score << "% (20% weight)\n";
    report << "  Test Coverage:    " << std::setw(6)
           << grade.test_coverage_score << "% (15% weight)\n";
    report << "  Code Quality:     " << std::setw(6)
           << grade.code_quality_score << "% (10% weight)\n";
    report << "\n";

    // Test Statistics
    report << "TEST STATISTICS:\n";
    report << "  Total Tests:      " << grade.coverage.total_tests << "\n";
    report << "  Passed:           " << grade.coverage.passed_tests << "\n";
    report << "  Failed:           " << grade.coverage.failed_tests << "\n";
    report << "  Pass Rate:        " << std::setprecision(1)
           << (grade.coverage.pass_rate * 100) << "%\n";
    report << "\n";

    // Performance Metrics
    report << "PERFORMANCE METRICS:\n";
    report << "  Mean Prediction:  " << std::setprecision(2)
           << grade.performance.mean_prediction_time_ms << " ms\n";
    report << "  P99 Prediction:   "
           << grade.performance.p99_prediction_time_ms << " ms\n";
    report << "  Mean Update:      "
           << grade.performance.mean_update_time_ms << " ms\n";
    report << "  Sustained Rate:   " << std::setprecision(0)
           << grade.performance.sustained_rate_hz << " Hz\n";
    report << "\n";

    // Strengths
    if (!grade.strengths.empty()) {
        report << "STRENGTHS:\n";
        for (const auto& strength : grade.strengths) {
            report << "  ✓ " << strength << "\n";
        }
        report << "\n";
    }

    // Areas for Improvement
    if (!grade.improvements_needed.empty()) {
        report << "IMPROVEMENTS NEEDED:\n";
        for (const auto& improvement : grade.improvements_needed) {
            report << "  • " << improvement << "\n";
        }
        report << "\n";
    }

    // Critical Issues
    if (!grade.critical_issues.empty()) {
        report << "⚠️  CRITICAL ISSUES:\n";
        for (const auto& issue : grade.critical_issues) {
            report << "  ✗ " << issue << "\n";
        }
        report << "\n";
    }

    report << "========================================\n";

    return report.str();
}

inline CoverageMetrics NavigationGrader::analyzeCoverage(
    const std::vector<TestResult>& results) {

    CoverageMetrics coverage;

    for (const auto& result : results) {
        coverage.total_tests++;

        if (result.passed) {
            coverage.passed_tests++;
        } else {
            coverage.failed_tests++;
        }

        // Categorize tests
        if (result.name.find("UKF") != std::string::npos ||
            result.name.find("StateVector") != std::string::npos) {
            coverage.ukf_tests_total++;
            if (result.passed) coverage.ukf_tests_passed++;
        }
        else if (result.name.find("RBPF") != std::string::npos ||
                 result.name.find("RBParticle") != std::string::npos) {
            coverage.rbpf_tests_total++;
            if (result.passed) coverage.rbpf_tests_passed++;
        }
        else if (result.name.find("Integration") != std::string::npos ||
                 result.name.find("Hierarchical") != std::string::npos) {
            coverage.integration_tests_total++;
            if (result.passed) coverage.integration_tests_passed++;
        }
        else if (result.name.find("Performance") != std::string::npos) {
            coverage.performance_tests_total++;
            if (result.passed) coverage.performance_tests_passed++;
        }
    }

    coverage.pass_rate = coverage.total_tests > 0 ?
        static_cast<double>(coverage.passed_tests) / coverage.total_tests : 0;

    return coverage;
}

inline void NavigationGrader::identifyStrengths(SystemGrade& grade) {
    if (grade.functionality_score >= 95) {
        grade.strengths.push_back("Excellent functional implementation");
    }
    if (grade.performance_score >= 90) {
        grade.strengths.push_back("Outstanding real-time performance");
    }
    if (grade.coverage.total_tests >= 100) {
        grade.strengths.push_back("Comprehensive test suite");
    }
    if (grade.coverage.pass_rate >= 0.95) {
        grade.strengths.push_back("High reliability with 95%+ test pass rate");
    }
    if (grade.performance.mean_prediction_time_ms < 1.0) {
        grade.strengths.push_back("Sub-millisecond prediction performance");
    }
    if (grade.performance.sustained_rate_hz >= 100) {
        grade.strengths.push_back("Achieves 100+ Hz sustained operation");
    }
}

inline void NavigationGrader::identifyImprovements(SystemGrade& grade) {
    if (grade.functionality_score < 80) {
        grade.improvements_needed.push_back("Improve core functionality implementation");
    }
    if (grade.performance_score < 75) {
        grade.improvements_needed.push_back("Optimize performance for real-time requirements");
    }
    if (grade.coverage.total_tests < 50) {
        grade.improvements_needed.push_back("Add more tests (current: " +
            std::to_string(grade.coverage.total_tests) + ", target: 100+)");
    }
    if (grade.coverage.ukf_tests_passed < grade.coverage.ukf_tests_total) {
        grade.improvements_needed.push_back("Fix failing UKF tests (" +
            std::to_string(grade.coverage.ukf_tests_total - grade.coverage.ukf_tests_passed) +
            " failures)");
    }
    if (grade.performance.mean_prediction_time_ms > 2.0) {
        grade.improvements_needed.push_back("Reduce prediction time (current: " +
            std::to_string(grade.performance.mean_prediction_time_ms) + "ms, target: <1ms)");
    }
}

inline void NavigationGrader::identifyCriticalIssues(SystemGrade& grade) {
    if (grade.coverage.pass_rate < 0.5) {
        grade.critical_issues.push_back("More than 50% of tests failing!");
    }
    if (grade.coverage.ukf_tests_passed == 0 && grade.coverage.ukf_tests_total > 0) {
        grade.critical_issues.push_back("All UKF tests failing - core functionality broken!");
    }
    if (grade.performance.sustained_rate_hz < 50) {
        grade.critical_issues.push_back("Cannot sustain minimum 50Hz operation!");
    }
    if (grade.quality.has_memory_leaks) {
        grade.critical_issues.push_back("Memory leaks detected!");
    }
    if (grade.quality.has_race_conditions) {
        grade.critical_issues.push_back("Race conditions detected!");
    }
    if (grade.functionality_score < 50) {
        grade.critical_issues.push_back("Critical functionality failures!");
    }
}

} // namespace Navigation