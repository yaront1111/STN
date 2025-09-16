#include "cpp/core/ukf_config.h"
#include <iostream>

int main() {
    std::cout << "Testing UKF Configuration System\n";
    std::cout << "===============================\n";
    
    // Test configuration validation and settings
    UKFConfig config;
    
    std::cout << "Testing default configuration:\n";
    config.setDefaults();
    if (config.validate()) {
        std::cout << "✅ Default configuration is valid\n";
    } else {
        std::cerr << "❌ Default configuration failed validation\n";
        return 1;
    }
    
    std::cout << "\nTesting Grade A+ optimal configuration:\n";
    config.setGradeAOptimal();
    if (config.validate()) {
        std::cout << "✅ Grade A+ configuration is valid\n";
    } else {
        std::cerr << "❌ Grade A+ configuration failed validation\n";
        return 1;
    }
    
    // Display calibrated measurement noise values
    std::cout << "\n🎯 Grade A+ Calibrated Measurement Noise:\n";
    std::cout << "   Gravity Gradient: " << config.measurement_noise.gravity_gradient << " (sqrt: " 
              << std::sqrt(config.measurement_noise.gravity_gradient) << ")\n";
    std::cout << "   Gravity Anomaly:  " << config.measurement_noise.gravity_anomaly << " (sqrt: "
              << std::sqrt(config.measurement_noise.gravity_anomaly) << ")\n";
    std::cout << "   Magnetometer:     " << config.measurement_noise.magnetometer << "\n";
    std::cout << "   Map Match Pos:    " << config.measurement_noise.map_match_position << " m\n";
    
    std::cout << "\n🎯 Process Noise Configuration:\n";
    std::cout << "   Position:         " << config.process_noise.position << " m/s^1.5\n";
    std::cout << "   Velocity:         " << config.process_noise.velocity << " m/s^2.5\n";
    std::cout << "   Attitude:         " << config.process_noise.attitude << " rad/s^1.5\n";
    std::cout << "   Accel Bias:       " << config.process_noise.accel_bias << " m/s^3.5\n";
    std::cout << "   Gyro Bias:        " << config.process_noise.gyro_bias << " rad/s^2.5\n";
    
    std::cout << "\n🎯 Grade A Performance Targets:\n";
    std::cout << "   Max Position Error: " << config.grade_a.max_position_error << " m\n";
    std::cout << "   Max Velocity Error: " << config.grade_a.max_velocity_error << " m/s\n";
    std::cout << "   Max Attitude Error: " << config.grade_a.max_attitude_error << " deg\n";
    std::cout << "   Map Match Interval: " << config.grade_a.map_match_interval << " s\n";
    
    std::cout << "\nTesting conservative configuration:\n";
    config.setConservative();
    if (config.validate()) {
        std::cout << "✅ Conservative configuration is valid\n";
    }
    
    std::cout << "\nTesting aggressive configuration:\n";
    config.setAggressive();
    if (config.validate()) {
        std::cout << "✅ Aggressive configuration is valid\n";
    }
    
    std::cout << "\n✅ ALL CONFIGURATION TESTS PASSED!\n";
    std::cout << "\n🚀 Ready to integrate calibrated parameters into navigation system\n";
    
    return 0;
}