#pragma once
#include "types.h"
#include <Eigen/Dense>

/**
 * IMU Data Processor
 * Handles raw IMU data preprocessing and calibration
 */
class IMUProcessor {
public:
    IMUProcessor() = default;
    
    /**
     * Process raw IMU sample with bias correction
     */
    ImuSample processSample(const ImuSample& raw, const State& state) {
        ImuSample corrected = raw;
        
        // Apply bias corrections
        corrected.acc_mps2 -= state.b_a;
        corrected.gyro_rps -= state.b_g;
        
        // Apply scale factor corrections (if available)
        // In production, would apply temperature compensation
        
        return corrected;
    }
    
    /**
     * Estimate noise characteristics from stationary data
     */
    void calibrateNoise(const std::vector<ImuSample>& stationary_data) {
        if (stationary_data.empty()) return;
        
        // Compute Allan variance for noise characterization
        // Simplified implementation
        Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
        
        for (const auto& sample : stationary_data) {
            acc_mean += sample.acc_mps2;
            gyro_mean += sample.gyro_rps;
        }
        
        acc_mean /= stationary_data.size();
        gyro_mean /= stationary_data.size();
        
        // Compute variance
        acc_noise_ = Eigen::Vector3d::Zero();
        gyro_noise_ = Eigen::Vector3d::Zero();
        
        for (const auto& sample : stationary_data) {
            Eigen::Vector3d acc_diff = sample.acc_mps2 - acc_mean;
            Eigen::Vector3d gyro_diff = sample.gyro_rps - gyro_mean;
            acc_noise_ += acc_diff.cwiseProduct(acc_diff);
            gyro_noise_ += gyro_diff.cwiseProduct(gyro_diff);
        }
        
        acc_noise_ /= stationary_data.size();
        gyro_noise_ /= stationary_data.size();
        
        acc_noise_ = acc_noise_.cwiseSqrt();
        gyro_noise_ = gyro_noise_.cwiseSqrt();
    }
    
    Eigen::Vector3d getAccNoise() const { return acc_noise_; }
    Eigen::Vector3d getGyroNoise() const { return gyro_noise_; }
    
private:
    Eigen::Vector3d acc_noise_{0.01, 0.01, 0.01};   // m/s²
    Eigen::Vector3d gyro_noise_{0.001, 0.001, 0.001}; // rad/s
};