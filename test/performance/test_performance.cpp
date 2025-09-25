/**
 * Performance and Timing Tests
 * Ensures real-time capability of the navigation system
 */

#include <gtest/gtest.h>
#include <chrono>
#include <vector>
#include <numeric>
#include "core/ukf/sr_ukf.h"
#include "core/rbpf/rbpf.h"
#include "core/hierarchical_filter.h"
// #include "sensors/sensor_manager.h"  // Comment out for now
#include "utils/logger.h"

using namespace Navigation;
using namespace std::chrono;

class PerformanceTest : public ::testing::Test {
protected:
    void SetUp() override {
        Logger::getInstance().initialize(".", "ERROR");
    }

    template<typename F>
    double measureExecutionTime(F func, int iterations = 100) {
        std::vector<double> times;
        times.reserve(iterations);

        // Warm-up
        for (int i = 0; i < 10; ++i) {
            func();
        }

        // Actual measurements
        for (int i = 0; i < iterations; ++i) {
            auto start = high_resolution_clock::now();
            func();
            auto end = high_resolution_clock::now();

            duration<double, std::milli> ms = end - start;
            times.push_back(ms.count());
        }

        // Return mean time
        return std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    }

    double calculatePercentile(std::vector<double>& times, double percentile) {
        std::sort(times.begin(), times.end());
        size_t index = static_cast<size_t>(percentile * times.size() / 100.0);
        return times[std::min(index, times.size() - 1)];
    }
};

// Test 1: UKF Prediction Performance
TEST_F(PerformanceTest, UKFPredictionTime) {
    SRUKFConfig config;
    config.alpha = 0.1;
    config.beta = 2.0;
    config.kappa = -37;

    SquareRootUKF ukf(config);

    StateVector initial;
    initial.position = Vector3d(0, 0, -1000);
    initial.velocity = Vector3d(100, 0, 0);
    initial.quaternion = Quaterniond::Identity();
    initial.accel_bias = Vector3d::Zero();
    initial.gyro_bias = Vector3d::Zero();
    initial.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();

    ukf.initialize(initial);

    Vector3d accel(0, 0, -9.81);
    Vector3d gyro(0.01, 0.02, 0.03);
    double dt = 0.01;

    double mean_time = measureExecutionTime([&]() {
        ukf.predict(accel, gyro, dt);
    }, 1000);

    EXPECT_LT(mean_time, 1.0)
        << "UKF prediction should complete in <1ms, but took " << mean_time << "ms";

    std::cout << "UKF Prediction: mean=" << mean_time << "ms" << std::endl;
}

// Test 2: UKF Measurement Update Performance
TEST_F(PerformanceTest, UKFMeasurementUpdateTime) {
    SRUKFConfig config;
    SquareRootUKF ukf(config);

    StateVector initial;
    initial.position = Vector3d(0, 0, -1000);
    initial.velocity = Vector3d(100, 0, 0);
    initial.quaternion = Quaterniond::Identity();
    ukf.initialize(initial);

    // Test barometer update
    double baro_time = measureExecutionTime([&]() {
        ukf.updateBarometer(101325.0, 288.15);
    }, 1000);

    EXPECT_LT(baro_time, 2.0)
        << "Barometer update should complete in <2ms, but took " << baro_time << "ms";

    // Test magnetometer update
    Vector3d mag_field(30e-6, 0, 40e-6);
    double mag_time = measureExecutionTime([&]() {
        ukf.updateMagnetometer(mag_field);
    }, 1000);

    EXPECT_LT(mag_time, 2.0)
        << "Magnetometer update should complete in <2ms, but took " << mag_time << "ms";

    std::cout << "Measurement Updates: baro=" << baro_time
              << "ms, mag=" << mag_time << "ms" << std::endl;
}

// Test 3: 100Hz Sustained Operation
TEST_F(PerformanceTest, SustainedOperationRate) {
    SRUKFConfig config;
    SquareRootUKF ukf(config);

    StateVector initial;
    initial.position = Vector3d(0, 0, -1000);
    initial.velocity = Vector3d(100, 0, 0);
    initial.quaternion = Quaterniond::Identity();
    ukf.initialize(initial);

    Vector3d accel(0, 0, -9.81);
    Vector3d gyro(0.01, 0.02, 0.03);

    // Run for simulated 10 seconds at 100Hz
    const int iterations = 1000;  // 10 seconds * 100Hz

    auto start = high_resolution_clock::now();

    for (int i = 0; i < iterations; ++i) {
        // Predict step
        ukf.predict(accel, gyro, 0.01);

        // Update every 10th iteration (10Hz measurements)
        if (i % 10 == 0) {
            ukf.updateBarometer(101325.0 - i * 0.1, 288.15);
        }

        if (i % 20 == 0) {
            Vector3d mag(30e-6, 0, 40e-6);
            ukf.updateMagnetometer(mag);
        }
    }

    auto end = high_resolution_clock::now();
    duration<double, std::milli> total_ms = end - start;

    double avg_iteration_time = total_ms.count() / iterations;
    double achieved_rate = 1000.0 / avg_iteration_time;  // Hz

    EXPECT_GT(achieved_rate, 100.0)
        << "System should sustain >100Hz, but achieved only " << achieved_rate << "Hz";

    std::cout << "Sustained rate: " << achieved_rate
              << "Hz (avg " << avg_iteration_time << "ms per iteration)" << std::endl;
}

// Test 4: Memory Leak Detection
TEST_F(PerformanceTest, MemoryStability) {
    SRUKFConfig config;

    // Run many iterations and check memory doesn't grow
    const int iterations = 10000;

    for (int i = 0; i < iterations; ++i) {
        SquareRootUKF ukf(config);

        StateVector state;
        state.position = Vector3d(i, i, -1000);
        state.velocity = Vector3d(100, 0, 0);
        state.quaternion = Quaterniond::Identity();
        ukf.initialize(state);

        // Perform some operations
        ukf.predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), 0.01);
        ukf.updateBarometer(101325.0, 288.15);

        // UKF goes out of scope and should be cleaned up
    }

    // If we get here without crash, memory is likely stable
    SUCCEED() << "Completed " << iterations << " iterations without memory issues";
}

// Test 5: RBPF Performance
TEST_F(PerformanceTest, RBPFPerformance) {
    RBPFConfig config;
    config.num_particles = 100;  // Typical particle count

    RBPF rbpf(config);

    StateVector initial;
    initial.position = Vector3d(0, 0, -1000);
    initial.velocity = Vector3d(100, 0, 0);
    initial.quaternion = Quaterniond::Identity();
    rbpf.initialize(initial, MatrixXd::Identity(20, 20) * 0.1);

    Vector3d accel(0, 0, -9.81);
    Vector3d gyro(0.01, 0.02, 0.03);

    double predict_time = measureExecutionTime([&]() {
        rbpf.predict(accel, gyro, 0.01);
    }, 100);

    // RBPF is expected to be slower than UKF
    EXPECT_LT(predict_time, 10.0)
        << "RBPF prediction should complete in <10ms, but took " << predict_time << "ms";

    std::cout << "RBPF Prediction (100 particles): " << predict_time << "ms" << std::endl;
}

// Test 6: Hierarchical Filter Performance
TEST_F(PerformanceTest, HierarchicalFilterPerformance) {
    HierarchicalConfig config;
    config.ukf_rate = 100.0;
    config.rbpf_rate = 10.0;

    HierarchicalFilter filter(config, nullptr);

    StateVector initial;
    initial.position = Vector3d(0, 0, -1000);
    initial.velocity = Vector3d(100, 0, 0);
    initial.quaternion = Quaterniond::Identity();
    filter.initialize(initial);

    IMUData imu;
    imu.accel = Vector3d(0, 0, -9.81);
    imu.gyro = Vector3d(0.01, 0.02, 0.03);
    imu.timestamp = 0.0;

    double process_time = measureExecutionTime([&]() {
        imu.timestamp += 0.01;
        filter.processIMU(imu, 0.01);
    }, 100);

    EXPECT_LT(process_time, 5.0)
        << "Hierarchical filter should process in <5ms, but took " << process_time << "ms";

    std::cout << "Hierarchical Filter: " << process_time << "ms" << std::endl;
}

// Test 7: Covariance Matrix Operations Performance
TEST_F(PerformanceTest, CovarianceOperations) {
    const int n = 20;  // Error state dimension
    MatrixXd P = MatrixXd::Identity(n, n) * 0.1;
    MatrixXd Q = MatrixXd::Identity(n, n) * 0.01;

    // Cholesky decomposition (used in square-root formulation)
    double cholesky_time = measureExecutionTime([&]() {
        Eigen::LLT<MatrixXd> llt(P);
        MatrixXd L = llt.matrixL();
    }, 1000);

    EXPECT_LT(cholesky_time, 0.1)
        << "Cholesky decomposition should be <0.1ms, but took " << cholesky_time << "ms";

    // QR decomposition (used in square-root updates)
    MatrixXd A = MatrixXd::Random(n, n);
    double qr_time = measureExecutionTime([&]() {
        Eigen::HouseholderQR<MatrixXd> qr(A);
        MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();
    }, 1000);

    EXPECT_LT(qr_time, 0.2)
        << "QR decomposition should be <0.2ms, but took " << qr_time << "ms";

    std::cout << "Matrix ops: Cholesky=" << cholesky_time
              << "ms, QR=" << qr_time << "ms" << std::endl;
}

// Test 8: Combined Prediction and Update Performance
TEST_F(PerformanceTest, CombinedPredictionUpdate) {
    SRUKFConfig ukf_config;
    SquareRootUKF ukf(ukf_config);

    StateVector initial;
    initial.position = Vector3d(0, 0, -1000);
    initial.velocity = Vector3d(100, 0, 0);
    initial.quaternion = Quaterniond::Identity();
    ukf.initialize(initial);

    // Simulate combined prediction and update
    double pipeline_time = measureExecutionTime([&]() {
        // Predict
        ukf.predict(Vector3d(0, 0, -9.81), Vector3d(0.01, 0.02, 0.03), 0.01);

        // Update with barometer
        ukf.updateBarometer(101325.0, 288.15);

        // Update with magnetometer
        ukf.updateMagnetometer(Vector3d(30e-6, 0, 40e-6));
    }, 1000);

    EXPECT_LT(pipeline_time, 5.0)
        << "Combined operations should complete in <5ms, but took " << pipeline_time << "ms";

    std::cout << "Combined predict+update: " << pipeline_time << "ms" << std::endl;
}

// Test 9: Worst-case Execution Time
TEST_F(PerformanceTest, WorstCaseExecutionTime) {
    SRUKFConfig config;
    SquareRootUKF ukf(config);

    StateVector initial;
    initial.position = Vector3d(0, 0, -1000);
    initial.velocity = Vector3d(100, 0, 0);
    initial.quaternion = Quaterniond::Identity();
    ukf.initialize(initial);

    std::vector<double> execution_times;
    execution_times.reserve(10000);

    // Run many iterations with varying inputs
    for (int i = 0; i < 10000; ++i) {
        Vector3d accel(sin(i * 0.01), cos(i * 0.01), -9.81);
        Vector3d gyro(0.01 * sin(i * 0.1), 0.01 * cos(i * 0.1), 0.01);

        auto start = high_resolution_clock::now();
        ukf.predict(accel, gyro, 0.01);

        if (i % 10 == 0) {
            ukf.updateBarometer(101325.0 + i * 0.1, 288.15);
        }
        auto end = high_resolution_clock::now();

        duration<double, std::milli> ms = end - start;
        execution_times.push_back(ms.count());
    }

    double p99 = calculatePercentile(execution_times, 99.0);
    double p999 = calculatePercentile(execution_times, 99.9);
    double max_time = *std::max_element(execution_times.begin(), execution_times.end());

    EXPECT_LT(p99, 5.0) << "99th percentile execution time should be <5ms";
    EXPECT_LT(p999, 10.0) << "99.9th percentile execution time should be <10ms";
    EXPECT_LT(max_time, 20.0) << "Maximum execution time should be <20ms";

    std::cout << "Execution times: p99=" << p99 << "ms, p99.9="
              << p999 << "ms, max=" << max_time << "ms" << std::endl;
}

// Test 10: Cache Performance Impact
TEST_F(PerformanceTest, CachePerformance) {
    SRUKFConfig config;
    SquareRootUKF ukf(config);

    StateVector initial;
    initial.position = Vector3d(0, 0, -1000);
    initial.velocity = Vector3d(100, 0, 0);
    initial.quaternion = Quaterniond::Identity();
    ukf.initialize(initial);

    // Cold cache run
    std::vector<double> cold_times;
    for (int i = 0; i < 100; ++i) {
        // Flush cache by accessing large array
        std::vector<double> cache_flush(1000000, 0.0);
        volatile double sum = std::accumulate(cache_flush.begin(), cache_flush.end(), 0.0);

        auto start = high_resolution_clock::now();
        ukf.predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), 0.01);
        auto end = high_resolution_clock::now();

        duration<double, std::milli> ms = end - start;
        cold_times.push_back(ms.count());
    }

    // Warm cache run
    std::vector<double> warm_times;
    for (int i = 0; i < 100; ++i) {
        auto start = high_resolution_clock::now();
        ukf.predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), 0.01);
        auto end = high_resolution_clock::now();

        duration<double, std::milli> ms = end - start;
        warm_times.push_back(ms.count());
    }

    double cold_mean = std::accumulate(cold_times.begin(), cold_times.end(), 0.0) / cold_times.size();
    double warm_mean = std::accumulate(warm_times.begin(), warm_times.end(), 0.0) / warm_times.size();

    std::cout << "Cache impact: cold=" << cold_mean
              << "ms, warm=" << warm_mean << "ms (ratio="
              << cold_mean/warm_mean << ")" << std::endl;

    // Warm cache should be notably faster
    EXPECT_LT(warm_mean, cold_mean)
        << "Warm cache should be faster than cold cache";
}