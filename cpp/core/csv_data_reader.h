#pragma once
#include "types.h"
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>

/**
 * CSV Data Reader for Testing Gravity Navigation
 * Reads flight data from CSV file
 */
class CSVDataReader {
public:
    struct DataRow {
        double t;
        double acc_x, acc_y, acc_z;
        double gyro_x, gyro_y, gyro_z;
        double lat_true, lon_true, alt_true;
        double gradient_xx, gradient_yy, gradient_zz;
        double gradient_xy, gradient_xz, gradient_yz;
        double anomaly_mgal;
        double temperature;
    };
    
    bool loadCSV(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "ERROR: Cannot open CSV file: " << filename << std::endl;
            return false;
        }
        
        std::string line;
        // Skip header
        std::getline(file, line);
        
        while (std::getline(file, line)) {
            DataRow row;
            std::stringstream ss(line);
            std::string field;
            std::vector<double> values;
            
            while (std::getline(ss, field, ',')) {
                values.push_back(std::stod(field));
            }
            
            if (values.size() >= 17) {
                row.t = values[0];
                row.acc_x = values[1];
                row.acc_y = values[2];
                row.acc_z = values[3];
                row.gyro_x = values[4];
                row.gyro_y = values[5];
                row.gyro_z = values[6];
                row.lat_true = values[7];
                row.lon_true = values[8];
                row.alt_true = values[9];
                row.temperature = values[10];
                row.gradient_xx = values[11];
                row.gradient_yy = values[12];
                row.gradient_zz = values[13];
                row.gradient_xy = values[14];
                row.gradient_xz = values[15];
                row.gradient_yz = values[16];
                row.anomaly_mgal = values[17];
                
                data_.push_back(row);
            }
        }
        
        file.close();
        std::cout << "Loaded " << data_.size() << " samples from CSV" << std::endl;
        return !data_.empty();
    }
    
    ImuSample getIMUSample(size_t index) const {
        if (index >= data_.size()) {
            return ImuSample();
        }
        
        const DataRow& row = data_[index];
        ImuSample sample;
        sample.t = row.t;
        sample.acc_mps2 = Eigen::Vector3d(row.acc_x, row.acc_y, row.acc_z);
        sample.gyro_rps = Eigen::Vector3d(row.gyro_x, row.gyro_y, row.gyro_z);
        sample.temperature_c = row.temperature;
        return sample;
    }
    
    GravityGradientTensor getGradient(size_t index) const {
        if (index >= data_.size()) {
            return GravityGradientTensor();
        }
        
        const DataRow& row = data_[index];
        GravityGradientTensor gradient;
        gradient.t = row.t;
        gradient.T << row.gradient_xx, row.gradient_xy, row.gradient_xz,
                      row.gradient_xy, row.gradient_yy, row.gradient_yz,
                      row.gradient_xz, row.gradient_yz, row.gradient_zz;
        gradient.R = Eigen::Matrix3d::Identity() * 1.0;  // 1 E noise
        return gradient;
    }
    
    double getAnomaly(size_t index) const {
        if (index >= data_.size()) return 0.0;
        return data_[index].anomaly_mgal;
    }
    
    Eigen::Vector3d getTruePosition(size_t index) const {
        if (index >= data_.size()) {
            return Eigen::Vector3d::Zero();
        }
        
        const DataRow& row = data_[index];
        return Eigen::Vector3d(row.lat_true, row.lon_true, row.alt_true);
    }
    
    size_t size() const { return data_.size(); }
    bool empty() const { return data_.empty(); }
    
private:
    std::vector<DataRow> data_;
};